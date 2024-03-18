# FOR API DEBUGGING ONLY. DO NOT RUN OTHERWISE
import requests
import json
import base64
from api.algo import generatePath

# url = "http://localhost:5000"
# x = requests.get(url)
# print(x.text)

url = "http://localhost:5000/test-image"
image_name = "images/" + "example.jpg"
image_type = 'jpg'
with open(image_name, "rb") as f:
    image = f.read()
payload = {
    "image": base64.b64encode(image).decode("utf-8"),
    "image_type": image_type,
    "obs": 0
}

headers = {'Content-type': 'application/json', 'Accept': 'text/plain'}
response = requests.post(url, headers=headers, json=payload)
try:
    data = response.json()
    print(data)
except:
    print("No response.")
'''
robotPos = [2, 2, 'N']
# obs = [[12, 2, 'S',1], [9, 18, 'W',2], [20, 13, 'S',3], [2, 8,'E',4], [5, 5, 'N', 5],[16, 6, 'E', 6], [12, 12, 'S', 7], [2, 12, 'E', 8]]
obs = [[12,1,'S',1]]
# obs = [[12, 2, 'N', 1], [11, 2, 'S', 2], [14, 2, 'S', 3], [12, 10, 'W', 4]]
print(generatePath(robotPos, obs))
'''