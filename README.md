# Detection of Localization Faults in the Central Place Foraging Algorithm.

## Current Version (06/27/23)

We use the Range and Bearing (RAB) sensor and actuator as a direct communication method between robots. The RAB actuator is responsible for broadcasting messages. The RAB sensor is responsible for receiving the broadcasted messages along with its range and bearing measurements. These measurements can be used to calculate the origin (location) of where the message was broadcasted from. 

Using the RAB actuator and sensor allows for a straightforward way of sending a robot’s estimated location and having other robots calculate and verify whether the sender’s estimated location matches a calculated location (using the range and bearing info) or is at least within a set tolerance to accommodate for noise and possible travel distance between steps. Useful configuration parameters for the RAB actuator and sensor are available such as sensor noise, range, message size, packet drop probability, and occlusion. Setting occlusion to true will prevent robots from being able to receive any messages broadcasted by a robot that is being “blocked” by another robot, in other words, it won’t get the message if it doesn’t have line-of-sight.

There are four processes that should happen sequentially and periodically (at a specified frequency). They are detailed as follows:

1.	Broadcasting the Robot's Location:
    -   In the loop function, retrieve the robot's current location using its sensors or other relevant information.
    -   Use the RAB actuator to broadcast the location information to other robots.
    -   Ensure that the message is correctly formatted and includes the necessary details such as the sender's ID and the coordinates.
2.	Processing the Broadcasted Locations:
    -   In the loop function, receive the location broadcasts from other robots using the RAB sensor.
    -   Compare the received coordinates against the robot's own calculated coordinates using the range and bearing information.
    -   Determine if the received locations are consistent with the robot's own calculations or if there's a potential localization fault.
    -   Store the information about perceived faults and associated robots for the next step.
3.	Broadcasting a Response:
    -   In the loop function, based on the comparison results, construct a response message in a specified format (e.g., a list) to inform the initial sender(s) about the correctness of their broadcasted location.
    -   Broadcast the response message using the RAB actuator.
4.	Processing the Responses:
    -   Implement a consensus mechanism to process the received responses from other robots.
    -   Maintain a record of which robots have voted and ensure that no robot can vote twice on the same robot.
    -   Once the voting cap is reached, analyze the votes for each robot to determine if there is a fault.
    -   If a fault is detected (i.e., a sufficient number of "wrong location" votes), the robot can admit to the fault and take appropriate action.
 
The consensus mechanism is simple, record the votes received from other robots in a queue and store the sender’s ID to make sure only one vote is recorded per robot. Every control step, check to see if the size of the queue meets the specified vote cap. Once reached, votes are popped from the queue and counted. If there is a tie, nothing is done and is treated as a pass (can continue operation). 
