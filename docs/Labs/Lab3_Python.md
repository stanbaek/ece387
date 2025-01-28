# 🔬 Lab3: Python

```{note}
Lab 3 is not ready yet!
```

# **Simplified Risk Game in Python**

## 📌 Objectives

- Students should be able to understand Object-Oriented Programming (OOP), including classes, attributes, and methods.
- Students should be able to explain inheritance and method overriding in OOP.
- Students should be able to implement game mechanics using programming logic in a game simulation.

---

## 📜 Overview of the Game

This simplified version of the Risk board game involves:
- Two players (a user and the computer).
- Recruiting armies with different unit types (`Footman`, `Archer`, `Knight`, and `SiegeMachine`).
- Taking turns to attack until one side is defeated.
- Using dice rolls to determine attack outcomes.

### Rules for This Simplified Risk Game

This version of Risk is a simplified, turn-based game involving two players: the **user** and the **computer**. Players recruit armies within a budget and take turns attacking each other until one side is eliminated. The game is played as follows:

---

### 1. Game Setup
- **Players**: There are two players:
  - **User**: You, the player.
  - **Computer**: Controlled by the program.
- **Budget**: Each player starts with a fixed number of coins to recruit their army. By default, this budget is 30 coins.
- **Unit Types**:
  - **Footman**: Cost: 1 coin, Health: 1, Hit Threshold: 5+
  - **Archer**: Cost: 2 coins, Health: 1, Hit Threshold: 4+
  - **Knight**: Cost: 3 coins, Health: 2, Hit Threshold: 3+
  - **Siege Machine**: Cost: 10 coins, Health: 3, Hit Threshold: 3+ (rolls two dice when attacking)

  The hit threshold is the minimum dice roll a unit needs to score a successful hit during an attack (e.g., a Footman hits on a roll of 5 or higher). It determines the unit's effectiveness in battle, with lower thresholds making a unit more likely to hit.

- **Recruitment**: 
  - Players recruit units randomly until their budget runs out.
  - A maximum of **2 Siege Machines** can be recruited per player.

---

### 2. Combat Mechanics
- **Turn-Based Gameplay**:
  - The **user** attacks first, followed by the **computer**.
  - Players alternate turns until one side is completely defeated.

- **Attacking**:
  - Players attack with one type of unit at a time (in the order: Siege Machine → Archer → Knight → Footman).
  - Each unit rolls dice to determine how many hits they score:
    - A unit scores a hit if the dice roll is greater than or equal to its **hit threshold**.
    - **Footman, Archer, and Knight** roll one die.
    - **Siege Machines** roll two dice.
  - The total number of hits is calculated and applied to the opposing player.

- **Defending**:
  - The defending player absorbs hits by assigning damage to random units in their army.
  - Units with zero health are eliminated and removed from the army.

---

### 3. End of Game
- The game ends when one player's army is completely defeated.
- The remaining player's army is declared the winner.
- The program displays:
  - The composition of the winning army.
  - A message announcing the winner.

---

### 4. Example Walkthrough
1. **Recruitment Phase**:
   - Both players recruit units based on their budget. For example:
     - User: 2 Footmen, 1 Archer, 1 Knight, 1 Siege Machine.
     - Computer: 3 Footmen, 2 Archers, 1 Knight.
   - The composition of each army is displayed before the battle begins.

1. **Battle Phase**:
   - **Turn 1**: 
     - User's Siege Machine rolls two dice and scores 1 hit. The hit is applied to a random unit in the computer's army.
     - Computer's Siege Machine rolls two dice and scores 2 hits. The hits are applied to the user's units.
   - **Subsequent Turns**:
     - Both players alternate attacking with their next unit types (e.g., Archers, Knights).
     - Damage is applied to the defending player's army after each attack.

1. **Game Over**:
   - The game ends when one player's army is eliminated. The program announces the winner and shows the remaining army of the victorious player.

---

This straightforward set of rules ensures that the focus remains on implementing and understanding the **object-oriented programming concepts** behind the game.

---

## 💻 Procedure

1. **Understand the Code**:
   - Read through each class and its methods.
   - Pay attention to the `TODO` sections.

1. **Implement the Missing Functionality**:
   - Follow the instructions in the `TODO` comments to complete the methods:

1. **Test the Implementation**:
   - Use the `test()` function provided in the code to test individual unit behaviors (e.g., `roll_attack` and `take_damage`).
   - Uncomment the `main()` function to play the full game after completing the `TODO` sections.

---

### Setup

1. Go to the [Setup Upstream Repository](UpstreamRepo) section to connect your repository to the instructor's (upstream) repository.  

1. Pull the latest changes from the upstream repository by running:  
    ```bash  
    git pull upstream main  
    ```  
    After completing this step, you should see a `lab3` directory containing Python scripts.  

1. Open **VS Code** by running the following command in your terminal:  
    ```bash  
    code .  
    ```  
---

### Classes and Responsibilities

```{hint}
You can find an example output in `example_output.txt`.  
```

#### **1. `Unit` Class**
- Represents a generic unit in the game.
- Attributes include:
  - `name`: The name of the unit.
  - `cost`: How much it costs to recruit this unit.
  - `health`: The unit's health points.
  - `hit_threshold`: The minimum dice roll needed for the unit to score a hit.
- **Methods**:
  - `roll_attack()`: Simulates a dice roll and determines if the unit scores a hit.
  - `take_damage(damage)`: Reduces the unit's health by the specified damage.
    - **TODO**: Subtract the damage value from `self.health` and ensure health doesn’t go below 0. Hint: Use the `max()` function to ensure health doesn’t drop below 0.
  - `isalive()`: Checks if the unit is still alive (health > 0). Hint: Check the value of `self.health` and return a boolean.
    - **TODO**: Return `True` if health > 0, else return `False`.

#### **2. Subclasses (`Footman`, `Archer`, `Knight`, `SiegeMachine`)**
- Inherit from the `Unit` class.
- Each unit type has specific attributes for `cost`, `health`, and `hit_threshold`.
- **SiegeMachine Special Ability**:
  - Rolls **two dice** instead of one for its attack.
  - **TODO**: Implement the `roll_attack` method in the `SiegeMachine` class to roll two dice and count the hits.

#### **3. `Player` Class**
- Represents a player (user or computer).
- Attributes include:
  - `name`: The player's name.
  - `budget`: Coins available for recruiting units.
  - `army`: List of units the player has recruited.
- **Methods**:
  - `recruit_units()`: Randomly recruits units within the player's budget.
    - **TODO**: Enforce the maximum number of Siege Machines (`MaxSiegeUnits`) allowed.
  - `attack(defender)`: Simulates an attack on another player.
    - **TODO**: Roll dice for units of the current type and calculate the total hits.
  - `resolve_damage(total_damage)`: Applies damage to the player's units.
    - **TODO**: Randomly pick units to take damage, remove units with zero health, and print eliminated units.

#### **4. `Risk` Class**
- Represents the overall game.
- Attributes:
  - `user`: The human player.
  - `computer`: The AI opponent.
- **Methods**:
  - `play()`: Runs the game loop until one player is defeated.
  - Prints the final outcome of the game.



### **Expected Outputs**

#### **Test Function**
When you run `test()`, you should see:
- Units rolling dice and scoring hits.
- Units taking damage and being eliminated when their health reaches 0.

#### **Main Game**
When you run the game:
- Both players will recruit armies based on their budget.
- Players will take turns attacking until one is defeated.
- The final result will display the surviving army composition for the winner.

---

## 🚚 **Deliverables**  

1. Complete all `TODO` sections in your code.  
1. Test your code thoroughly to ensure it works as expected.  
1. Once your code is complete, save the output to a file by running:  
    ```bash  
    python risk.py > output.txt  
    ```  
1. Push both your code and the output file (`output.txt`) to your GitHub repository.  
1. Submit the **Lab3** assignment on Gradescope.  
---

Good luck, and enjoy coding! 🎯