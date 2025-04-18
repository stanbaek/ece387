# 🚀 Final Project


## NOT READY YET



```{attention}
Read this page thoroughly before you start working on this final project.   
```


## 📌 Objectives  

- Students will integrate concepts from earlier modules to design and implement a complete robotic system capable of handling complex tasks using ROS2.  
- Students will demonstrate proficiency in robotic systems by building task-specific machines with onboard computing.  
- Students will apply knowledge from across the course to develop, test, and deliver a functional final project.


## 📜 Agenda

The final project brings together everything you've learned throughout the course. It challenges you to design and build a complete robotic system using ROS2 that can solve a real-world task with multiple components working together. This project emphasizes embedded, task-specific robotics and is where you apply your accumulated knowledge in a hands-on, functional way.

Your robot will operate in the maze in the lab, using a combination of technologies from previous modules. LIDAR will help it stay centered between walls, the IMU will handle precise turns (90°, 180°, 360°), and OpenCV will detect and respond to randomly placed stop signs. Additionally, you'll use AprilTags to guide the robot through the maze —navigating based on tag ID and distance to determine whether it should turn left, right, around, or stop at its goal.

```{important}
If you notice any discrepancies in the project description or grading criteria, the correct information will follow this order of priority: (1) Instructor messages via Teams, (2) this Course Web, (3) Gradescope, and (4) the project overview slides. The Course Web will always be updated immediately to reflect any changes, and outdated instructions will be clearly crossed out.
```

##  Synopsis

🏴‍☠️ Ahoy, crew! The infamous pirate captain, Dr. Baek, has discovered an ancient scroll that reveals the location of a hidden treasure island. But there’s a catch—the treasure is locked deep inside a deadly maze! 

According to the scroll, the maze is filled with mysterious symbols called AprilTags. These symbols are the only clues that lead to the treasure. Miss one, or read it wrong, and... well, let’s just say you won’t be making it back.

But instead of risking life and limb, Captain Baek wants to use some good ol' 21st-century tech. That’s where you come in. Your mission: build a robotic system that can explore the maze on its own, follow the AprilTags, and find the treasure.

Here is what the scroll says
    - You must follow the walls in the maze. 
    - Do not stop at windows or look out of the windows as they are so deadly.
    - At AprilTag ID 0, you must turn left. Make sure you make a 270$^\circ$ clockwise turn. 
    - At AprilTag ID 1, you must stop and pause for 5 seconds then make a 90$^\circ$ left turn.
    - At AprilTag ID 2, you must turn right. Make sure you make a 90$^\circ$ clockwise turn. 
    - At AprilTag ID 3, you must turn left. Make sure you make a 90$^\circ$ counterclockwise turn. 
    - At a stop sign, you must stop from 0.3 meters from the stop sign, then make a left turn. The treasure chest is located around there.  Now you should look around to find the exact location of it. The treasure chest is buried right under AprilTag ID 4.
    

## 🎮 Final Project Gamesmanship

1. Demo and coding (40 points)
    - Read the presentation and final report requirements before you start coding.
    - Start early to earn early-bird bonus points; delay may jeopardize timely completion.
    - Use the code from Labs 10 and 11 as foundations, but avoid implementing your final project in these lab files. Instructors will review the code inside the `final_project` folder. 
    - Employ `rqt`, `ros2 topic`, and `rviz` extensively for debugging high-level behaviors. 
    - **Cease work for the final report**
        - Your analysis in the report is much more important than completing the maze. 
        - Balance time spent on coding and demo; don't sacrifice report quality for demos. Don't lose 30 points on the report to earn 15 demo points.
        - If your robot reaches only halfway to the Level 1 goal point, the deduction will be approximately 5-10 points.

1. Presentation (20 points)
    - Utilize visual aids a lot! Figures, tables, and graphs are more helpful than words.
    - Ensure you **discuss everything** in the presentation section.
    - Adhere to the 5-minute time limit; practice for effective delivery. Your talk will be stopped at the 6-minute mark, and credit will not be given for parts not discussed. Adhering to the time limit for presentations is a basic etiquette. Even at professional conferences, talks can be cut short if they exceed the allotted time.
    - You are strongly recommended to practice your talk. Students who gave very short presentations to avoid exceeding the time limit often did not discuss enough details and lost even more points. So, practice! Even experienced engineers practice for their conference presentations.

1. Report (40 points)
    - Thoroughly Read the template and **do not miss anything in the template**.
    - Use figures and tables to support your analysis and results.
    - While in-person demos are accepted, ensure submission of video demos aligned with the plots in your report.

## 🛠️ Requirements

### Timeline

1. L36 0700: Design Presentation slides (Gradescope and Instructors)
    - Submit your `MS PowerPoint pptx` file to your instructor NLT L36 0700. Your slides will be played on your instructor's PC for smooth transitions between speakers. Make sure to send a pptx file and not a Keynote file or Google Slides unless your instructor has approved it.
    - Additionally, submit the PDF version of your presentation file on Gradescope no later than L36 0700. 
    - **_No grace days_** can be used for the PowerPoint slides.
1. L36: Design Presentations
1. L39 0700: Live demo Due   
    - Late Demos: You can use grace days, but all products must be submitted NLT T40 2359 (by the Dean's policy). 
1. T40 2359: Final report & Code
    - No grace days can be used. All products must be submitted by midnight on T40. 

### Requirements

1. Add comprehensive comments throughout your code. Emphasize the importance of thorough commenting.
1. Avoid delays, loops, waits, or sleeps in code.  Since you need to use a timer, adding delays, loops, or waits will interfere with the timer. The timer it self is actually a loop running forever.


### Deductions
- Deductions of up to 5 points will apply if delays or loops exist within the controller. 
- A penalty of up to 5 points will be incurred for poor coding practices, such as inadequate comments or the excessive use of hard-coded numbers instead of variables or enumerated types.


## 🎬 Demonstrations 

Demonstrations will be accomplished on lesson 39 in the maze. Points will be deducted for failed checkpoints (e.g., does not stop and turn within approximately 2 meters of AprilTag 0). The final rubric is below and a total of **40 points** assigned to the demonstration:

- Wall following [10 points]: The robot should follow the walls without hitting them. A deduction of 1 point for each wall collision will be applied. No more than 5 points will be deducted if the robot reaches the end goal ($\pm$30 cm from the yellow line).  
- Stop sign [10 points]: The robot should stop at the yellow line. The bottom plate of the robot should cover any part of the yellow line. Once the treasure chest is located, print out the current location of the robot and stay there forever. 
- AprilTags [20 points]: The robot should stop at the orange line before making any turns. The bottom plate of the robot should cover any part of the orange line. Then, the robot should make a turn. Although the exact angle of the turns will not be accessed, you might want to get precise turns as it affect the overall performance of the robot.



## 👩‍🏫 Design Presentation (20 Points)

Provide a **5-minute** presentation followed by approximately 2 minutes for Q&A, covering the following topics:

```{note}
Make extensive use of visual aids, such as a maze picture, flowcharts, and diagrams. Avoid filling slides with excessive text; instead, keep your audience engaged and focused on listening to you.
```

1. **[3 Points]** Purpose: 
    - Briefly describe the problem.
    - Discuss the project requirements.
1. **[10 Points]** Design:
    - This section accounts for 50% of the presentation grade.
    - Discuss the details of your design. For example,
        - How to make turn decisions at intersections.
        - How to keep track of the robot's coordinates.
        - How to handle misclassifications.
        - Specify the timers you plan to use, their intended purposes, and operating frequencies.
        - Include a clearly legible finite state machine.
    - Outline the data you intend to collect for experimental analysis and explain your data collection method.
1. **[4 Points]** Debugging and testing:
    - Provide detailed information on debugging and testing methods. Explain how your methods enhance debugging and testing efficiency.
    - Describe high-level debugging strategies for unexpected robot behavior in the maze, including identification and resolution of issues like misclassification or getting trapped at a corner.
    - Exclude discussions related to compile-time debugging, such as syntax errors and register configurations.
1. **[3 Points]** Questions: 
    - Prepare to answer questions about your design choices.
    - Be ready to address inquiries regarding implementation of additional functionality in your project.
    - Note that the question component extends beyond the 6-minute presentation by an additional 2 minutes.
1. **Timing**:
    - Points will be deducted for exceeding the 6-minute limit.
    - **The presentation will be halted at the 7-minute mark, and credit will not be given for parts not discussed.**  Therefore, **practice!**
    - Emphasize the importance of professional timing in a presentation, considering its impact on the audience and subsequent speakers.
1. **Submission**:
    - Submit your `MS PowerPoint pptx` file to your instructor **NLT L34 0700**. Your slides will be played on your instructor's PC for smooth transitions between speakers. Make sure to send a pptx file and not a Keynote file unless your instructor has approved it.
    - Additionally, export your PowerPoint file to a **pdf** file and submit it on Gradescope by L34 0700. **Be sure to select questions and pages to indicate where your responses are located. Failure to do so will result in point deductions.** Refer to the instruction gif inside the Final Report section for guidance. 
    - **_No grace days_** can be used for the PowerPoint slides.


    ```{attention}
    Submit your MS PowerPoint pptx file to your instructor. Additionally, export your PowerPoint file to a pdf file and submit it on Gradescope. Select questions and pages to indicate the locations of your response. 
    ```

## 📈 Final Report (40 Points)

Refer to the example report, `ECE387_Project Report Template.docx`, available in Teams under Files > Class Materials, for detailed guidance on report content. 

```{note}
Even if your demos are not successful, whether you use grace days or not, ensure you discuss everything based on the progress you have made.
```

- **[5 Points]** Introduction/Purpose:
    - Describe the problem.
    - Discuss the requirements.
    - Outline any assumptions made at the project's outset.
- **[10 Points]** Design: 
    - Discuss design choices for each level, including values for PWM_AVERAGE, $k_p$, and $k_i$.
    - Specify the timers used, their purposes, and the operating frequencies.
    - Explain the data collected for experiment analysis and detail your data collection method.
- **[5 Points]** Debugging and testing:
    - Provide comprehensive details on debugging and testing methods.
    - Explain how your methods enhance debugging and testing efficiency.
    - Describe how you identified and resolved unexpected robot behaviors.
    - Exclude discussions related to compile-time debugging, such as syntax errors.
- **[15 Points]** Analysis and Results:
    - _**This is the most essential part of the final project. You are expected to provide high-quality engineering analysis.**_
    - Your analysis should be based on collected data, utilizing figures and tables. For example, evaluate the robot's performance in Level 2 by employing step responses. Base your analysis on data and avoid relying on visual observations.
    - Discuss results for each level, including measurements and plots.
    - Evaluate the time taken for your robot to complete tasks.
    - If unresolved issues exist, discuss potential solutions with more time.
    - Highlight any unique features that make your robot stand out.
    - Which tasks were you unable to complete, and what were the reasons for the inability?
    - Discuss ongoing issues and problems and how you addressed them.
    - Did you do anything unique to make your robot better than others?
- **[5 Points]** Conclusion
    - Conclude your work
    - Concisely summarize the results. 
    - Emphasize the overall result of the project.

```{attention}
Submit your report on Gradescope. Be sure to select questions and pages to indicate where your responses are located. Failure to do so will result in point deductions.
```

```{image} ./figures/Proj_GradescopeSubmission.gif
:width: 740
:align: center
```
<br>

## 🚚 Deliverables

```{note}
Unlike other assignments, any points earned above the total project points (250) can be applied to your final course grade.
```

### Deliverable 1: [50 Points] Design Presentation
- **[5 Points]** Purpose
- **[25 Points]** Design
- **[15 Points]** Debugging and testing
- **[5 Points]** Questions

### Deliverable 2: [100 points + $\alpha$] Demo and Code
- **[15 Bonus Points]**: 
    - **[5 Points]** Early turkey
    - **[10 Points]** 5 points for each level
    - **[5-15 points]** Final Race
    
- **Deductions:**
    - **[-10 Points]:** 5 points per level deducted for delays or loops in ISR.
    - **[-10 Points]:** 5 points per level deducted for controller code in main instead of ISR.
    - **[-10 Points]:** Up to 5 points per level deducted for bad coding practices (e.g., poor comments, hard-coded numbers).
    - Uncompilable code results in a grade of 0 for the level. 
- 🏇 **Final Race**
    - L39 during your section. We will measure completion time for the Level 2 demo.  
    - Same requirements as Level 2 apply: reach the goal and stop before hitting the wall.
    - You will have two rounds: 2 minutes for the first round and 1 minute for the second round.
    - A run is considered valid as long as it starts within the given timeframe.
    - Code changes are allowed within the 3-minute timeframe. While you can reflash your program multiple times within the 3 minutes, it is highly recommended to use LCD and switches for adjustments.
    - If you are next in line, you need to stand by on the deck. There won't be any wait time once the previous turn is complete.
    - Prizes:
        - 1st place: 15 bonus points and name on the lab plaque.
        - 2nd place: 12 bonus points
        - 3rd and 4th places: 10 bonus points
        - 1st place in each section if not in the 1st - 4th places in class: 5 bonus points

### Deliverable 3: [100 Points + $\alpha$] Report 
- **[10 Points]** Introduction
- **[20 Points]** Design 
- **[20 Points]** Debugging and testing
- **[40 Points]** Analysis and Results
- **[10 Points]** Conclusion
- **[IP Points]** Extra points for the best paper in class!

### Deliverable 4: [0 Points] Return your robot
- Return your robot in person to the instructors or Mr. Hall by <span style="color:blue"> Wed 11 Dec 1500.</span>
- Ensure there are no loose parts, and tighten all screws before returning your robot.
- **[-20 Points]** Up to 20 points may be deducted for loose parts or not returning the robot in person on time. 

<br>
<br>


