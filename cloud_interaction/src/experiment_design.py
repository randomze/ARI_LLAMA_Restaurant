#!/usr/bin/env python3
import asyncio
from statemachine import StateMachine, State
import time
import rospy
import requests
import actionlib
import queue
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText
from pal_navigation_msgs.msg import GoToPOIAction, GoToPOIGoal
from std_msgs.msg import String
import os
from datetime import datetime


# This experiment consists in 3 phases.
# 1. The robot moves to the queue, and welcomes the customer
# 2. The robot moves to the table, and takes the order
# 3. The robot displays the order to the participant and farewell them


class CloudTurnTakingSpeechInteraction:
    def __init__(
        self, participant_id, *, server_ip: str = "localhost", server_port: int = 5000
    ):
        server_uri = f"http://{server_ip}:{server_port}"
        print("[SYSTEM] Initializing CloudTurnTakingSpeechInteraction")
        self.ollama_server_take_order_food = f"{server_uri}/take_order_food"
        self.ollama_server_sit_customer = f"{server_uri}/sit_customer"
        self.ollama_server_take_order_drink = f"{server_uri}/take_order_drink"
        self.ollama_server_take_order_dessert = f"{
            server_uri}/take_order_dessert"
        self.number_retries = 0
        self.max_number_retries = 3

        self.FLAG_END = 0
        self.flag_set_subscriber = rospy.Subscriber(
            "/set_flag_end", String, self.set_flag_end
        )

        # Initialize the user text subscriber
        self.user_text_subscriber = rospy.Subscriber(
            "/dialogflow_text", String, self.user_text_callback
        )
        self.user_text_publisher = rospy.Publisher(
            "/dialogflow_text/string_msg", String, queue_size=10
        )
        self.user_message_queue = queue.Queue()
        self.can_listen = False

        # Subscribe to the TTS action server
        self.tts_action_client = actionlib.SimpleActionClient("/tts", TtsAction)
        self.tts_action_client.wait_for_server(rospy.Duration(10))
        self.tts_language = "en_GB"
        print("[SYSTEM] Finished initializing TTS")
        print("[SYSTEM] Finished Initializing CloudTurnTakingSpeechInteraction")

        self.logger = Logger(participant_id)

    def set_flag_end(self, msg):
        self.FLAG_END = 1
        return

    def user_text_callback(self, msg):
        print(f"[USER] - {msg.data}")

        if self.can_listen:
            self.user_message_queue.put(msg.data)

    def say_text(self, text):
        # Play the greeting over TTS and block until it's done speaking
        self.tts_action_client.send_goal(
            TtsGoal(rawtext=TtsText(text=text, lang_id=self.tts_language))
        )
        self.tts_action_client.wait_for_result()

    def process_llm_response(self, response):
        user_id = 0

        print("Printing Response\n\n", response)
        print("[+++++] FLAG END IS SET TO ", self.FLAG_END)
        try:
            if "|" in response:
                tag, content = response.split("|")
                if "finished" in tag.replace(" ", ""):
                    print("[System] Detected Intent")
                    return 1, user_id, content
                else:
                    return 1, user_id, content
            elif "<finished>" in response:
                print("[System] Detected Intent")
                return 1, user_id, content
            else:
                return 1, user_id, content
        except:
            print("I'm in the exception")
            content = response
            return 1, user_id, content

    def interact_with_user(self, ollama_server, user_id):
        # Greet the customers
        print(f"[CHATBOT]> Greeting the user")

        start_time_request = time.time()
        request = {"input": "<start>"}
        post_req = requests.post(ollama_server, json=request)
        response_data = post_req.json()

        response = response_data.get("response")
        call_type = response_data.get("call_type")
        print("CALL TYPE", call_type)
        elapsed_time = response_data.get("elapsed_time")

        total_time = time.time() - start_time_request
        latency_time = (total_time - elapsed_time) / 2

        self.logger.log_message(
            user_id, "PROCESSING TIME | " + call_type, elapsed_time, response
        )
        self.logger.log_message(user_id, "LATENCY | " + call_type, latency_time, "")
        self.logger.log_message_new(user_id, call_type, "", total_time, response, 0)

        _, _, greeting = self.process_llm_response(response)

        # TODO TTS for the greeting
        print(f"[CHATBOT]> {greeting}")
        # Play the greeting over TTS and block until it's done speaking
        self.tts_action_client.send_goal(
            TtsGoal(rawtext=TtsText(text=greeting, lang_id=self.tts_language))
        )
        self.tts_action_client.wait_for_result()

        # Listen for the customer order

        status, _, response = self.listen_customer_response(ollama_server, user_id)
        return status, user_id, response

    def listen_customer_response(self, ollama_server, user_id):
        rospy.sleep(0.1)
        self.can_listen = True
        user_response = ""
        while user_response == "":
            self.user_text_publisher.publish(String("Please tell me your order"))
            while self.user_message_queue.empty():
                rospy.sleep(0.1)

            user_response = self.user_message_queue.get()
        self.can_listen = False
        print(f"[USER {user_id}] - {user_response}")

        return self.interpret_customer_response_reply(
            ollama_server, user_id, user_response
        )

    def interpret_customer_response_reply(self, ollama_server, user_id, order):
        start_time_request = time.time()

        request = {"input": order}
        post_req = requests.post(ollama_server, json=request)

        response_data = post_req.json()

        response = response_data.get("response")
        call_type = response_data.get("call_type")
        elapsed_time = response_data.get("elapsed_time")

        total_time = time.time() - start_time_request
        latency_time = (total_time - elapsed_time) / 2

        status, _, text = self.process_llm_response(response)

        self.logger.log_message(
            user_id, "PROCESSING TIME | " + call_type, elapsed_time, response
        )
        self.logger.log_message(user_id, "LATENCY | " + call_type, latency_time, "")
        self.logger.log_message_new(user_id, call_type, order, total_time, text, 0)

        if status == 0:  # Successfully understood the client's order
            # Communicate that we did not understand and ask them to try again
            self.tts_action_client.send_goal(
                TtsGoal(rawtext=TtsText(text=text, lang_id=self.tts_language))
            )
            self.tts_action_client.wait_for_result()
            return 0, user_id, text
        elif status == 1:  # Did not understand, please try again
            # Counte the number of failures
            self.number_retries += 1

            # if the number of failure exceeds the number of retries
            # go to fallback option
            # if self.number_retries > self.max_number_retries:
            #     return 1, user_id, ""

            # TODO TTS TO RETRY ORDER
            # Play the greeting over TTS and block until it's done speaking
            print(f"[CHATBOT]> {text}")
            self.tts_action_client.send_goal(
                TtsGoal(rawtext=TtsText(text=text, lang_id=self.tts_language))
            )
            self.tts_action_client.wait_for_result()

            print("[********] printing again self.FLAG_END", self.FLAG_END)
            if self.FLAG_END == 1:
                content = response
                print("FLAG END SET TO TRUE")
                return 0, user_id, "INTERACTION ENDED"

            return self.listen_customer_response(ollama_server, user_id)


class Logger:
    def __init__(self, participant_id):
        self.log_file_path = self.setup_logging(participant_id)

    def setup_logging(self, participant_id):
        script_dir = os.path.dirname(
            os.path.realpath(__file__)
        )  # Directory of the current script
        log_dir = os.path.join(script_dir, "../logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
            print("Done")
        else:
            print("Directory already exists")
        return os.path.join(log_dir, f"participant_{participant_id}.log")

    def log_message_new(
        self, participant_id, type_call, time, message_user, message_chatbot, delay
    ):
        type_call = type_call.strip()
        message_user = str(message_user).strip()
        message_chatbot = message_chatbot.strip()
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(self.log_file_path, "a") as log_file:
            if "|" in message_chatbot:
                message_chatbot = message_chatbot.split("|")[1]
            log_file.write(
                f"{timestamp} | {type_call} | {time} | {
                    message_user} | {message_chatbot} | {delay}\n"
            )

    def log_message(self, participant_id, type_call, time, message):
        with open(self.log_file_path, "a") as log_file:
            log_file.write(f"{type_call} | {time} | {message}\n")


class RobotBasedExperiment(StateMachine):
    # States
    # waitingCustomer = State('WaitingCustomer')
    greetingCustomer = State("GreetingCustomer", initial=True)
    takingOrder = State("TakingOrder")
    deliverOrder = State("DeliverOrder", final=True)

    # customer_at_queue = waitingCustomer.to(greetingCustomer)
    customer_seated = greetingCustomer.to(takingOrder)
    order_taken = takingOrder.to(deliverOrder)

    def __init__(self):
        super().__init__()
        self.participant_id = rospy.get_param("~participant_id", 0)
        self.server_ip = rospy.get_param("~server_ip", "localhost")
        self.server_port = rospy.get_param("~server_port", 5000)
        print(self.participant_id)

        self.condition, self.type_of_order = self.read_condition_participant()

        self.dish = None

        if "offloaded" in self.condition:
            self.participant_id = str(self.participant_id) + "_offloaded"
            self.turntaker = CloudTurnTakingSpeechInteraction(
                self.participant_id,
                server_ip=self.server_ip,
                server_port=self.server_port,
            )
        else:
            raise ValueError(
                "Please modify the conditions_order.txt file and make sure you are running the offloaded speech engine."
            )

    def read_condition_participant(self):
        condition = ""
        lines = ""

        curr_dir = os.path.dirname(os.path.abspath(__file__))
        filename_conditons = os.path.join(curr_dir, "conditions_order.txt")

        with open(filename_conditons, "r") as f:
            # Read the first line and remove it
            participant_conds = f.readline()
            participant_id = participant_conds.split(",")[0]
            if participant_id != self.participant_id:
                print("Participant ID does not match")
            condition = participant_conds.split(",")[1]
            type_of_order = participant_conds.split(",")[2]
            # Remove the first line from the file
            lines = f.readlines()
        # Write the remaining lines to the file
        with open(filename_conditons, "w") as f:
            f.writelines(lines)
        return condition, type_of_order

    def nav_go_to_poi(self, name):
        move_client = actionlib.SimpleActionClient(
            "/poi_navigation_server/go_to_poi", GoToPOIAction
        )
        move_client.wait_for_server()
        rospy.loginfo("POI navigation client ready")

        goal = GoToPOIGoal()
        goal.poi.data = name
        rospy.loginfo("Sending goal")
        move_client.send_goal(goal)
        wait = move_client.wait_for_result()
        res = move_client.get_result()
        rospy.loginfo("[INFO] Navigation result: %s", res)

    async def on_enter_waitingCustomer(self):
        print("Waiting for customer to be seated")

    async def on_enter_greetingCustomer(self):
        print("Processing Customer: ", self.participant_id)
        print("The type of order is: ", self.type_of_order)
        await asyncio.sleep(0.5)

    async def on_enter_takingOrder(self):
        print("I'm in taking order")

        # Taking order

        if "GPT" in self.participant_id:
            print("THERE'S GPT")
            self.turntaker.interact_with_user(
                self.participant_id, "order_" + self.type_of_order
            )
        else:
            print("THERE IS NO GPT")
            if self.type_of_order.strip() == "food":
                self.turntaker.interact_with_user(
                    self.turntaker.ollama_server_take_order_food, self.participant_id
                )
            elif self.type_of_order.strip() == "drink":
                self.turntaker.interact_with_user(
                    self.turntaker.ollama_server_take_order_drink, self.participant_id
                )
            elif self.type_of_order.strip() == "dessert":
                self.turntaker.interact_with_user(
                    self.turntaker.ollama_server_take_order_dessert, self.participant_id
                )

        # TOCHECK
        # self.dish = "Salad"
        print(f"Order {self.dish} taken")
        await self.order_taken()

    async def on_enter_deliverOrder(self):
        print(f"Showing the {self.dish} to the customer")
        print("End of the interaction")

    # async def on_exit_waitingCustomer(self):
    #     self.nav_go_to_poi("queue")
    #     await asyncio.sleep(1)

    async def on_exit_greetingCustomer(self):
        self.nav_go_to_poi("table_1")
        print("Going to table_1")
        await asyncio.sleep(1)

    async def on_exit_takingOrder(self):
        print("Cooking the order")
        self.turntaker.say_text("Thank you!")
        self.nav_go_to_poi("queue")
        await asyncio.sleep(1)


async def run_experiment():
    print("[INFO] Experiment Started")
    experiment = RobotBasedExperiment()
    print("[INFO] Sending a customer at the queue")
    # await experiment.customer_at_queue()
    await experiment.customer_seated()


if __name__ == "__main__":
    rospy.init_node("experiment_design", anonymous=True)
    asyncio.run(run_experiment())
