# 🚀 Final Project

```{attention}
Read this page thoroughly before you start working on this final project.   
```

## 📌 Objectives  

- Students will integrate concepts from earlier modules to design and implement a complete robotic system capable of handling complex tasks using ROS2.  
- Students will demonstrate proficiency in robotic systems by building task-specific machines with onboard computing.  
- Students will apply knowledge from across the course to develop, test, and deliver a functional final project.

---

## 📜 Overview  

In this final project, you’ll combine everything you’ve learned throughout the course to design and implement a complete robotic system using ROS2. The project challenges you to create a robot that solves real-world tasks, requiring multiple components to work together seamlessly.  

Your robot will navigate a maze in the lab, employing technologies from previous modules:  
- **LIDAR:** Keeps the robot centered between walls.  
- **IMU:** Handles precise turns (90°, -90°, 270°).  
- **OpenCV:** Detects and reacts to randomly placed stop signs.  
- **AprilTags:** Guides the robot through the maze using tag IDs and distances to decide actions like turning or stopping at specific locations.  

```{important}
If you notice any discrepancies in the project description or grading criteria, the correct information will follow this order of priority: (1) Instructor messages via Teams, (2) this Course Web, (3) Gradescope, and (4) the project overview slides. The Course Web will always be updated immediately to reflect any changes, and outdated instructions will be clearly crossed out.
```

## Story  

Ahoy, crew! 🏴‍☠️ The infamous pirate captain, Dr. Baek, has uncovered an ancient scroll that reveals the location of a hidden treasure island. But there’s a catch—the treasure is locked deep inside a deadly maze! 

According to the scroll, the maze is marked with enigmatic symbols known as AprilTags. These cryptic signs hold the key to locating the treasure. Miss one, or read it wrong, and... well, let’s just say you won’t be making it back.

Rather than risking life and limb in the perilous maze, the legendary pirate captain, Dr. Baek, is turning to cutting-edge technology to solve the mystery. Your task is to design and build a robotic system capable of navigating the maze independently, decoding the AprilTags, and uncovering the treasure's hidden location.

Here’s what the scroll reveals about navigating the maze:  
1. **Follow the walls in the maze at all times.**  
2. **Avoid stopping at or looking out windows—those are deadly!**  
3. **React to AprilTags:**  
   - **Tag ID 0:** Turn left (make a 270° clockwise turn).  
   - **Tag ID 1:** Stop for 5 seconds, then make a 90° left turn.  
   - **Tag ID 2:** Turn right (make a 90° clockwise turn).  
   - **Tag ID 3:** Turn left (make a 90° counterclockwise turn).  
   - **Stop Signs:** Stop 0.3 meters before the sign, then turn left. Look around to locate the treasure chest, which is buried under **AprilTag ID 4**.  

Once you locate the treasure chest, send Captain Baek the maze map so he can claim the loot!

---

## 🎮 Final Project Gamesmanship

### 1. Demo & Coding (40 points)  
- Review the presentation and final report requirements **before** you begin coding.  
- **Start early**—delays can put your completion at risk.  
- Use the code from Labs 10 and 11 as a reference, but **do not** implement your project within those lab files. Instructors will evaluate the code inside your `final_project` folder.  
- Take advantage of debugging tools like `rqt`, `ros2 topic`, and `rviz` to fine-tune high-level behaviors.  
- **Prioritize your final report:**  
  - Your analysis in the report is more important than completing the maze.  
  - Manage your time carefully—don’t sacrifice report quality for extra demo points. Losing 10 points on the report to earn 5 points in the demo isn’t worth it!  

### 2. Presentation (20 points)  
- **Make visuals a priority!** Figures, tables, and graphs often communicate concepts better than lengthy explanations.  
- **Cover all required details** in your presentation.  
- Stick to the **5-minute limit**—practice to refine your delivery. At the 6-minute mark, your talk will be stopped, and any content beyond that **won’t** be graded. Time management is key, just as it is in professional conferences.  
- **Practice, practice, practice.** Short presentations that lack detail often result in lost points. Even experienced engineers practice for their conference presentations.

### 3. Final Report (40 points)  
- Carefully **follow the template** and ensure all required sections are included.  
- Strengthen your analysis with figures and tables to clearly present your findings.  
- If you opt for an in-person demo, **also** submit a video demo that aligns with your report’s plots to support your documentation.  

---

## 🛠️ Requirements

### Timeline  

1. **L36 0700: Design Presentation Slides (Gradescope & Instructors)**  
    - Submit your **PowerPoint (.pptx)** file to your instructor by **L36 0700**.  
    - Your slides will be displayed on your instructor’s PC for seamless transitions between speakers.  
    - Make sure to submit a **pptx file**, not Keynote or Google Slides, unless your instructor has specifically approved it.  
    - Additionally, upload a **PDF version** of your slides on Gradescope by the same deadline.  
    - **No grace days** are available for presentation slides—late submissions won’t be accepted.  

1. **L36: Design Presentations**: Prepare to present your project design during **L36**.  

1. **L39 0700: Live Demo Due**  
    - You may use **grace days**, but all submissions must be completed by **T40 2359**, as per the Dean’s policy.  

1. **T40 2359: Final Report & Code**  
    - **No grace days** are available—everything must be submitted by **midnight (T40 2359)**.

### Coding Requirements  
1. Add detailed comments to your code for clarity.  
1. Avoid delays, loops, waits, or sleeps, as they interfere with the timer functionality.  

### Penalties  
- Up to 5 points will be deducted for delays or loops in the controller code.  
- Poor coding practices, such as inadequate comments or hard-coded values, may result in up to 5 points being deducted.  

---
## 🎬 Demonstrations  

Below is the final grading rubric, with a total of **40 points** assigned to the demonstration:  

1. **Wall Following (10 points):**  
  - The robot must follow the maze walls without colliding.  
  - Each wall collision results in a **1-point deduction**.  
  - However, if the robot successfully reaches the final goal (yellow line), no more than **3 points** will be deducted.  

1. **Stop Sign (8 points):**  
  - The robot must stop at the **yellow line**, with its bottom plate covering any part of it.  
  - Once the treasure chest is found, the robot should **print its current location** and remain there indefinitely.  

1. **AprilTags (15 points):**  
  - The robot must stop at the **orange line** before turning.  
  - Its bottom plate must cover at least part of the orange line before executing a turn.  
  - Precise turns aren't directly graded, but improving turn accuracy will **boost overall performance**.  

1. **Map Generation (5 points):**  
  - As the robot explores the maze, it must generate a **map**.  
  - This map should be included in your final report.  

1. **Gamepad Control (2 points):**  
  - You must be able to **relinquish control** to the robot when needed.  

## 👩‍🏫 Design Presentation (20 Points)  

Your presentation will last **5 minutes**, followed by a **2-minute Q&A** session. Be sure to cover the following topics:  

```{note}  
Use **visual aids** like diagrams, flowcharts, and maze images to enhance clarity.  Avoid filling slides with excessive text; instead, keep your audience engaged and focused on listening to you.
```  

1. **Purpose [3 Points]**  
    - Briefly explain the **problem** your robot is solving.  
    - Discuss the **project requirements**.  

2. **Design [10 Points]**  
    - **This is the most heavily weighted section (50% of the presentation grade).**  
    - Detail your design approach, including:  
        - **Using timers** to avoid loops and delays.  
        - **Tracking turning angles** for accurate maneuvering.  
        - **Determining the robot’s coordinates** at the treasure chest.  
        - Including a **clearly legible finite state machine diagram**.  

1. **Debugging & Testing [4 Points]**  
    - Explain how your testing methods improve **debugging efficiency**.  
    - Describe **strategies for handling unexpected robot behavior**, such as:  
        - Misreading **AprilTags** or the **stop sign**.  
        - Identifying and resolving navigation errors.  
    - **Do not** include basic syntax or build error discussions—focus on **high-level troubleshooting techniques**.  

4. **Questions [3 Points]**  
    - Prepare to **defend your design choices**.  
    - Be ready to discuss **possible enhancements** to your project.  
    - **Q&A extends beyond the 5-minute presentation**, adding another **2 minutes** for discussion.  

5. **Timing & Practice**  
    - **Points will be deducted** for exceeding the **5-minute limit**.  
    - **Presentations will be stopped at the 6-minute mark**—content beyond that won’t be graded.  
    - **Practice your delivery** to balance detail and conciseness—timing matters!  

6. **Submission**  
    - **By L36 0700**, submit your **PowerPoint (.pptx) file** to your instructor.  
    - **Additionally**, export your slides as a **PDF** and submit them on Gradescope.  
    - **No grace days** are allowed for presentation slides. **Late submissions incur a 3-point penalty.**  

```{attention}  
On Gradescope, select questions/pages to indicate where your responses are located—failure to do so will result in point deductions.  
```  

## 📈 Final Report (40 Points)  

Refer to **`ECE387_Project Report Template.docx`** (available on Teams under **Files > Class Materials**) for detailed guidelines.  

```{note}  
Even if your demo is not successful, whether you use grace days or not, ensure you discuss everything based on the progress you have made.
```  

1. **Introduction/Purpose [5 Points]**
    - Clearly define the **problem** and project **requirements**.  

2. **Design [10 Points]**: Discuss key design elements, including:  
  - Image resolution, frame rate, and sensor choices.  
  - **Controller design**, including finite state machine details.  
  - Include an **RQT graph** with **legible plots**.  

3. **Analysis & Results [20 Points]**  
    - **This is the most crucial section—your engineering analysis matters!**  
    - Base findings on collected **data** rather than visual observations.  
    - Evaluate performance of **vision sensors, LiDAR, and IMU**.  
    - Provide **measurements, plots, and time-based evaluations** for each task.  
    - Discuss **any unresolved challenges** and propose solutions.  
    - Highlight **unique features** that set your robot apart.  
    - Clearly explain **incomplete tasks** and **why they weren’t achieved**.  

4. **Conclusion [5 Points]**  
    - Summarize your findings and project outcomes concisely.  

```{attention}  
On Gradescope, select questions/pages to indicate where responses are located—failure to do so **will result in deductions**.  
```  

```{image} ./figures/Proj_GradescopeSubmission.gif  
:width: 740  
:align: center  
```  
---

## 🚚 Deliverables  

### **Deliverable 1: [20 Points] Design Presentation**  
- **[3 Points]** Purpose  
- **[10 Points]** Design  
- **[4 Points]** Debugging & Testing  
- **[3 Points]** Questions  

### **Deliverable 2: [40 points] Demo & Code**  
#### **Demo Breakdown**
- **[10 Points]** Wall following  
- **[8 Points]** Stop sign behavior  
- **[15 Points]** AprilTags navigation  
- **[5 Points]** Map generation  
- **[2 Points]** Gamepad control  
#### **Code Submission:**
- Push the code from the master computer to GitHub.
- If there is additional code on the robot, compress it using tar and gzip (`.tar.gz` format) and send it to your instructor. Make sure to research how to properly create the compressed archive.

#### **Deductions**  
- **[-5 Points]:** Delays or loops in ISR.  
- **[-5 Points]:** Poor coding practices (e.g., lack of comments, excessive hard-coded values).  

### **Deliverable 3: [40 Points + Bonus] Final Report**  
- **[5 Points]** Introduction  
- **[10 Points]** Design  
- **[20 Points]** Analysis & Results  
- **[5 Points]** Conclusion  
- **[Bonus Points]** Extra credit for the **best report in class**!  

