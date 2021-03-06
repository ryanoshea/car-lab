# Car Lab

## ELE 302: Building Real Systems, Princeton, 2015

During the Spring semester of my Junior year at Princeton, I took ELE 302 and built an autonomous electric vehicle. Starting with an RC electric hobby car, we stripped out the radio module and motor and steering controllers, then built replacements from scratch, building in additional behaviors not present in the original car. The platform we were given was a Cypress PSoC 3, plus as many blank generalized PCBs and electronic components we could find, make, or order. 

The car itself was contained mostly in a base with plastic standoffs emerging from it, upon which a flat acrylic platform was mounted, which we could attach components to. The electrical interfaces we were provided with were as follows:

- The positive and negative terminals of a reversible DC motor.
- A data line to the steering motor allowed for control via a PWM signal, where a middle frequency caused the wheels to become straight and frequencies on either side turned the wheels a proportional angle.  

**TL;DR: [video of the car](https://www.youtube.com/watch?v=azzE5iQZgSc) // [photos of the car](https://www.flickr.com/photos/rinoshea/albums/72157651516510906)**

A photo of the final car:

![](https://raw.githubusercontent.com/ryanoshea/car-lab/master/images/final%20car.jpg)

You can find more photos [on Flickr](https://www.flickr.com/photos/rinoshea/albums/72157651516510906).

### Stage 1: Speed Control

We designed and built the circuitry to make the car travel straight at 3 ft/sec to within 2% accuracy on flat ground and 10% accuracy on an incline and decline.

*Note: Details of our implementation for this stage of the course have been left out of this page and repository, so as not to provide possible solutions to students taking the course in the future.* 

### Stage 2: Line Following

We attached a VGA camera on a laser-etched acrylic mast, aimed at the floor. Using it, we tracked a black line on a light floor and controlled the car's steering to cause it to follow the line around a track. The car with the mast, camera, and relevant circuitry is shown below:

![](https://raw.githubusercontent.com/ryanoshea/car-lab/master/images/line%20following%20car.jpg)

*Note: Details of our implementation for this stage of the course have been left out of this page and repository, so as not to provide possible solutions to students taking the course in the future.*

### [Stage 3: Autonomous Obstacle Avoidance](https://github.com/ryanoshea/car-lab/blob/master/Stage%203%20Report%20-%20Autonomous%20Obstacle%20Avoidance.pdf)

The third stage was an independent project. We chose to implement obstacle avoidance. We mounted two sonar rangefinders to a rotating platform on the top of the car to monitor the car's surroundings. Using that data, we instructed the car to drive forward and avoid obstacles in the process, navigating around them forward and backwards if necessary. The surroundings data was also broadcast over a wireless RS-232 connection to a laptop. The PSoC code used to achieve this navigation can be found in [`main.c`](https://github.com/ryanoshea/car-lab/blob/master/main.c). The Matlab script we wrote to parse and plot the surroundings data is [`surroundings_mapping_script.m`](https://github.com/ryanoshea/car-lab/blob/master/surroundings_mapping_script.m). 

**A video of the car in action can be found [here](https://www.youtube.com/watch?v=azzE5iQZgSc).**

To achieve this behavior, we added quite a few features to the circuitry and programming of the car, which I've summarized below. You can see a more complete elaboration in the Stage 3 final report linked to in the section header.

- We added an H-bridge to our power MOSFET-based throttle system. While before we used a PWM signal from the PSoC to open and close a transistor, we added a second signal that controlled the direction of the motor, along with the throttle control.
- We removed the mast and camera mount structure from the car and added a custom 3-segment rectangular elevated platform made from laser-etched acrylic. On top of this, we mounted two large acrylic gears: one over the exact center of the robot and one off to the side powered by a small continuous rotation servo.
- On the center gear in the above assembly, we mounted two Parallax *Ping)))* sensors facing directly out from the center of the gear, positioned at opposite edges of the gear. Through the center of the gear, we positioned a slip ring that allowed the sensors on the gear to rotate endlessly on the gear while maintaining a stationary electrical connection to the car's electronics below. These sensors rotated at 50 rpm, driven by the servo attached to the other gear. By measuring quickly as they rotated, we could gather a picture of the car's surroundings every half-rotation, or 100 times per minute (full refresh every 0.6 seconds). However, we designed the control algorithm to update itself more quickly than this, always using all of the most recently updated data and taking advantage of new data as it came in, rather than waiting for a full update of the data. 
- To accurately measure the angle at which each distance reading was taken, we relied on two things: first, the continuous rotation servo could be counted on the rotate at a relatively uniform speed; second, we mounted a magnet on one of the gear's teeth that passed over a Hall effect sensor once per rotation. We used what we knew about the gear's rotation and the time since the magnet last passed over the sensor to determine the sensor's orientation relative to the car for each range measurement.
- The sensor data we received, which came at various angles around the car, was divided into 18 angular slices of space surrounding the car (each with a 20 degree field of view). So, ultimately, the car's internal representation of its surroundings was 18 vectors arranged radially around the car in even increments. Any measurements made within a given slice of the circle (which we referred to as a "bin") during a single pass through that slice were averaged together into one reading representing the distance an obstacle positioned at that angle relative to the front of the car. That averaged measurement corresponded to the length of the vector positioned at the center of that bin.  
- We took advantage of the PSoC's UART module by connecting it to an XBee wireless RS-232 transmitter. We used this to transmit the car's internal representation of its surroundings, as measured by the *Ping)))* sensors, to a laptop with the receiver attached to a serial port. A Matlab script ran on the laptop to continuously plot this incoming data. The data displayed was placed in a moving average filter to minimize noise in the plots. 

#### Obstacle Avoidance Algorithm

Every 200ms, an interrupt was fired on the PSoC to update the steering of the car based on its environment. There is a response only if there is an object detected within 1 meter, otherwise the car will drive straight at a speed of 2.5 feet per second. When the interrupt is fired, the car searches through each of the bins (20-degree slices of the area around the car) and stores the two closest objects and their bin numbers, which correspond to their approximate positions relative to the car. The algorithm designed to simulate an awareness that objects seen in adjacent bins are not seen as distinct objects (e.g. a wall is a wall even if it is picked up in more than one bin). If the second closest object is outside of a threshold distance, the car acts only on the first object, (assuming it is within the threshold). If both objects are within the threshold, an “effective bin” is generated by a weighted average of the bin numbers, with the weights generated by the distances to the objects. For example, if two nearby objects are detected directly in front of and to the right of the car, the effective bin will indicate to the car that it should behave as if there is a single object at a 45-degree angle to its front-right. The effective bin gives us a good idea of the general direction of objects in the car’s view that it should respond to. After the effective bin is generated, the car goes through a manually specified set of instructions to react to an object in that bin. Namely, it steers away from them. Specifically, objects behind the car lead to the car driving forward, directly away from the object (this includes steering the car to move directly away). If the obstacle is near the front of the car but off to the side (within 1 meter), the wheels will point in a direction away from the object in an attempt to steer out of the way of the object without stopping to back up. If that object moves closer, to within a secondary inner threshold (0.65 meters), the car will stop its forward motion, back up, turn away from the obstacle, and then continue on. Because steering out of the way doesn’t do much to help with an object directly in front of the car, any object directly in front will result in the car reversing direction and turning the wheels in such a way as to put the object to its side. The car adapts its throttle when dealing with obstacles to move away from them effectively. We made many fine adjustments to how quickly the car should move in a variety of circumstances, but for example, the car moves away from objects substantially faster if they are closer than 0.4 meters, while it adopts a slower pace while avoiding objects farther away. We noticed that our car is best used with static objects, although dynamic objects are also detectable and avoidable, within reasonable restrictions. 

The car is able to detect and avoid objects of varying size, ranging from walls to the legs of chairs. The car seems to be able to detect nearly all surfaces, though it was unable to pick up soft foam balls, which absorb the ultrasound signals broadcast by the car. Because the car cannot see objects that are very low to the ground (below 3 inches in height), it sometimes collides with people’s feet, since it will detect the person’s legs but not their protruding feet. However, in most cases, it responds well to human obstacles, even those with soft, loose clothing. During our demo, a professor stepped out onto the course about 2 feet in front of the car in motion, and it successfully avoided a collision with her. The car’s overall range is from about 3 meters on all sides of the car right up to directly touching the car. The sensors work down to about 3 cm, meaning that even objects that are just next to the car are well within the sensing range (although any object 3 cm from the sensor would have already collided with the car). It can see a vertical range from about 3 inches above the floor to about 3 feet above it, depending on the distance the object is from the sensor. Overall, we were very pleased with the car’s ability to avoid both static and moving obstacles both large and small. It ​very  rarely came into contact with an obstacle during our testing, and we found its behavior to be accurate, responsive, and intuitive (i.e. after fine-tuning its behavior, the car seemed to move out of the way of obstacles it was presented with in a way that one would expect it to, if it were controlled by a human).

A detail of the sonar platform is shown below, followed by an example of the surroundings data plot:

![](https://raw.githubusercontent.com/ryanoshea/car-lab/master/images/sensor%20platform.jpg)

![](https://raw.githubusercontent.com/ryanoshea/car-lab/master/images/surroundings%20plot.png)