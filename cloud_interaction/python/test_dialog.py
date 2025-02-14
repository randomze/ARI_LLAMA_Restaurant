#!/usr/bin/env python3
import rospy
import actionlib
import queue
from pal_interaction_msgs.msg import TtsAction, TtsGoal, TtsText
from std_msgs.msg import String
import time

class TestDialog:
    def __init__(self):
        print("[SYSTEM] Initializing Test Dialog")

        # Initialize the user text subscriber
        self.user_text_subscriber = rospy.Subscriber("/dialogflow_text", String, self.user_text_callback)
        self.user_text_publisher = rospy.Publisher("/dialogflow_text/string_msg", String, queue_size=10)
        self.user_message_queue = queue.Queue()
        self.can_listen = False

    
    def user_text_callback(self,msg):
        print(f"[USER] - {msg.data}")

        if self.can_listen:
            self.user_message_queue.put(msg.data)

        
    def listen(self):
        rospy.sleep(0.1)
        print("[SYSTEM] Start listening...")
        self.can_listen = True
        response = ""
        start_time = time.time()
        while response == "":
            self.user_text_publisher.publish(String("Please speak..."))
            while self.user_message_queue.empty():
                rospy.sleep(0.1)
                curr_time = time.time()
                print(f"[+] {curr_time}")
                if curr_time - start_time > 5:
                    break

            response = self.user_message_queue.get()
        self.can_listen = False
        print("[SYSTEM] Finished listening")

    def spinner(self):
        print("[SYSTEM] spinner active...")
        speak = input("Do you want to speak? (Y/N)")
        while speak.lower() == "y":
            self.listen()
            speak = input("Do you want to speak? (Y/N)")
        else:
            return

if __name__ == "__main__":
    rospy.init_node("test_dialog", anonymous=True)
    tester = TestDialog()
    tester.spinner()