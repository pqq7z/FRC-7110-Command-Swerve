# Setup

There are a couple simple steps to complete the setup for this code. 
**all steps are required**

1. update all the CAN on all CAN devices to required ids found in Constants.h
2. Power on the robot
3. Read the printed angles from the CANCoders found in the Rio Log. 
4. Add or subtract the required amount from each CANCoder to make their adjusted value 180. 
5. Deploy the updated code to the Robot. 

At this point the robot should be working and any futher issue will have to be resolved at that point. 

# Other Info

- This code is not very well documented however I believe I can explain most parts if there are any questions. 
- I did my best to increase performance any chance I got and I think it is possible that I may have decreased readablitly in some aspects. (Probably not that big of a deal)
- It should have a working auto however the implementation WPILib provides is not clarified and does not make that much sense to me. I might just implement it in my own way but that's for later. 
- There is some debug tools that I added that print to the smartdashboad, turn rate, heading, field relative. 
- there are also buttons to zero the gyro and toggle field relative which are the left and right bumper respectivly. 

I recomend that this code gets reviewed at somepoint as I may have made some dumb mistakes along the way. 