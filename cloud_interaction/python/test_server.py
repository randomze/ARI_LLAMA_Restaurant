#!/usr/bin/env python3

import rospy
import requests


class TestServer:

    def __init__(self):
        self.ollama_server_sit_customer = "http://localhost:5000/sit_customer"
        print(f"Trying to connect to {self.ollama_server_sit_customer}")

    def test_start_query(self, ollama_server):
        request = {'input': '<start>'}
        post_req = requests.post(ollama_server, json = request)

            # Extracting the JSON response
        response_data = post_req.json()

        # Extracting the specific fields from the response
        response = response_data.get('response')
        call_type = response_data.get('call_type')
        elapsed_time = response_data.get('elapsed_time')

        # Now you can use the extracted fields as needed
        print("Response:", response)
        print("Call Type:", call_type)
        print("Elapsed Time:", elapsed_time)

    def execute(self):
        self.test_start_query(self.ollama_server_sit_customer)

if __name__ == '__main__':

    tester = TestServer()
    print("Testing the communication with the server")
    tester.execute()