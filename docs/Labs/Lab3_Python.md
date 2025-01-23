# 🔬 Lab3: Python



# **Simplified Risk Game in Python**

---

## **Objective**

This lab will help you learn the following key programming concepts in Python:
1. **Object-Oriented Programming (OOP):**
   - Classes, attributes, and methods.
   - Inheritance and method overriding.
2. **Game Simulation:**
   - Implementing game mechanics using programming logic.
3. **Using TODOs for Active Learning:**
   - You'll implement specific parts of the game by completing the `TODO` sections in the code.

---

## **Overview of the Game**

This simplified version of the Risk board game involves:
- Two players (a user and the computer).
- Recruiting armies with different unit types (`Footman`, `Archer`, `Knight`, and `SiegeMachine`).
- Taking turns to attack until one side is defeated.
- Using dice rolls to determine attack outcomes.




### **Rules for This Simplified Risk Game**

This version of Risk is a simplified, turn-based game involving two players: the **user** and the **computer**. Players recruit armies within a budget and take turns attacking each other until one side is eliminated. The game is played as follows:

---

### **1. Game Setup**
- **Players**: There are two players:
  - **User**: You, the player.
  - **Computer**: Controlled by the program.
- **Budget**: Each player starts with a fixed number of coins to recruit their army. By default, this budget is 30 coins.
- **Unit Types**:
  - **Footman**: Cost: 1 coin, Health: 1, Hit Threshold: 5+
  - **Archer**: Cost: 2 coins, Health: 1, Hit Threshold: 4+
  - **Knight**: Cost: 3 coins, Health: 2, Hit Threshold: 3+
  - **Siege Machine**: Cost: 10 coins, Health: 3, Hit Threshold: 3+ (rolls two dice when attacking)
- **Recruitment**: 
  - Players recruit units randomly until their budget runs out.
  - A maximum of **2 Siege Machines** can be recruited per player.

---

### **2. Combat Mechanics**
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

### **3. End of Game**
- The game ends when one player's army is completely defeated.
- The remaining player's army is declared the winner.
- The program displays:
  - The composition of the winning army.
  - A message announcing the winner.

---

### **Example Walkthrough**
1. **Recruitment Phase**:
   - Both players recruit units based on their budget. For example:
     - User: 2 Footmen, 1 Archer, 1 Knight, 1 Siege Machine.
     - Computer: 3 Footmen, 2 Archers, 1 Knight.
   - The composition of each army is displayed before the battle begins.

2. **Battle Phase**:
   - **Turn 1**: 
     - User's Siege Machine rolls two dice and scores 1 hit. The hit is applied to a random unit in the computer's army.
     - Computer's Siege Machine rolls two dice and scores 2 hits. The hits are applied to the user's units.
   - **Subsequent Turns**:
     - Both players alternate attacking with their next unit types (e.g., Archers, Knights).
     - Damage is applied to the defending player's army after each attack.

3. **Game Over**:
   - The game ends when one player's army is eliminated. The program announces the winner and shows the remaining army of the victorious player.

---

### **Key Differences from the Original Risk Game**
- This version simplifies recruitment and combat:
  - There is no board or territorial control.
  - Armies are recruited randomly, and players have fixed budgets.
  - Combat is resolved by alternating turns, with no simultaneous attacks or strategic maneuvers.
- Unit roles and dice mechanics are adjusted for simplicity.

---

This straightforward set of rules ensures that the focus remains on implementing and understanding the **object-oriented programming concepts** behind the game.





---

## **Setup**

1. Open your Python development environment (e.g., VS Code or PyCharm).
2. Copy and paste the provided code into a new `.py` file.
3. Follow the instructions in each **`TODO`** section to complete the missing functionality.

---

## **Classes and Responsibilities**

### **1. `Unit` Class**
- Represents a generic unit in the game.
- Attributes include:
  - `name`: The name of the unit.
  - `cost`: How much it costs to recruit this unit.
  - `health`: The unit's health points.
  - `hit_threshold`: The minimum dice roll needed for the unit to score a hit.
- **Methods**:
  - `roll_attack()`: Simulates a dice roll and determines if the unit scores a hit.
  - `take_damage(damage)`: Reduces the unit's health by the specified damage.
    - **TODO**: Subtract the damage value from `self.health` and ensure health doesn’t go below 0.
  - `isalive()`: Checks if the unit is still alive (health > 0).
    - **TODO**: Return `True` if health > 0, else return `False`.

#### **Tasks for Students**
1. **Complete the `take_damage` method**:
   - Use the `max()` function to ensure health doesn’t drop below 0.
   ```python
   self.health = max(self.health - damage, 0)
   ```
2. **Complete the `isalive` method**:
   - Check the value of `self.health` and return a boolean.

---

### **2. Subclasses (`Footman`, `Archer`, `Knight`, `SiegeMachine`)**
- Inherit from the `Unit` class.
- Each unit type has specific attributes for `cost`, `health`, and `hit_threshold`.
- **SiegeMachine Special Ability**:
  - Rolls **two dice** instead of one for its attack.
  - **TODO**: Implement the `roll_attack` method in the `SiegeMachine` class to roll two dice and count the hits.

#### **Tasks for Students**
1. **Modify `roll_attack` in `SiegeMachine`**:
   - Use a loop to roll two dice.
   - Count how many rolls are greater than or equal to `hit_threshold`.
   ```python
   hits = 0
   for _ in range(2):
       roll = random.randint(1, 6)
       print(f"{self.name} rolls {roll}.")
       if roll >= self.hit_threshold:
           hits += 1
   return hits
   ```

---

### **3. `Player` Class**
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

#### **Tasks for Students**
1. **Complete `recruit_units`**:
   - Skip recruiting Siege Machines if `siege_count >= MaxSiegeUnits`.
   ```python
   if isinstance(unit, SiegeMachine) and siege_count >= self.MaxSiegeUnits:
       continue
   ```
2. **Complete `attack`**:
   - Loop through `self.army` and roll dice for units of the current type (`army_type`).
   - Add hits to `total_hits` and print results.
   ```python
   if isinstance(unit, army_type) and unit.isalive():
       hits = unit.roll_attack()
       total_hits += hits
       print(f"{unit} scores {hits} hit(s).")
   ```
3. **Complete `resolve_damage`**:
   - While `total_damage > 0`, randomly pick units from `self.army` using `random.choice()`.
   - Apply 1 damage to the selected unit, remove dead units, and print their names.
   ```python
   while total_damage > 0 and self.army:
       unit = random.choice(self.army)
       unit.take_damage(1)
       if not unit.isalive():
           print(f"{unit.name} has been eliminated!")
           self.army.remove(unit)
       total_damage -= 1
   ```

---

### **4. `Risk` Class**
- Represents the overall game.
- Attributes:
  - `user`: The human player.
  - `computer`: The AI opponent.
- **Methods**:
  - `play()`: Runs the game loop until one player is defeated.
  - Prints the final outcome of the game.

---

## **Steps to Complete the Lab**

1. **Understand the Code**:
   - Read through each class and its methods.
   - Pay attention to the `TODO` sections.

2. **Implement the Missing Functionality**:
   - Follow the instructions in the `TODO` comments to complete the methods:
     - `Unit.take_damage`
     - `Unit.isalive`
     - `SiegeMachine.roll_attack`
     - `Player.recruit_units`
     - `Player.attack`
     - `Player.resolve_damage`

3. **Test the Implementation**:
   - Use the `test()` function provided in the code to test individual unit behaviors (e.g., `roll_attack` and `take_damage`).
   - Uncomment the `main()` function to play the full game after completing the `TODO` sections.

---

## **Expected Outputs**

### **Test Function**
When you run `test()`, you should see:
- Units rolling dice and scoring hits.
- Units taking damage and being eliminated when their health reaches 0.

### **Main Game**
When you run the game:
- Both players will recruit armies based on their budget.
- Players will take turns attacking until one is defeated.
- The final result will display the surviving army composition for the winner.

---

## **Submission**
1. Complete all `TODO` sections.
2. Test your code thoroughly.
3. Submit the completed `.py` file to the course platform.

---

### **Learning Outcomes**
By completing this lab, you will:
1. Understand how to design and implement OOP systems.
2. Learn how to use inheritance to extend functionality.
3. Gain experience with simulating game mechanics using Python.

Good luck, and enjoy coding! 🎯