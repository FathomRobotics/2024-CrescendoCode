
# Welcome to the Fathom Robotics 2024 Crescendo Code Repository
This is where the code that we will use for competitions will be stored
## How to Setup Your Environment on your Computer

> When setting up your environment, we recommend the usage of the
> Anaconda package manager alongside the PyCharm IDE and GitHub Desktop!

### Python Installation
Firstly, we will need to install Python 3.11, you can install it from the [Anaconda Package Manager](https://www.anaconda.com/download) or directly from python.org
### RobotPy Installation
Once you have installed Python onto your computer you will want to install RobotPy! To do this, we will need to open a command prompt tab with Python in the path variables of the terminal ([How To With Anaconda](https://conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html#activating-an-environment) + [How To With Local Python](https://www.simplilearn.com/tutorials/python-tutorial/python-path)). Once the terminal is open, execute the following code:
`python -m pip install robotpy[all]`
This will install the Python library named robotpy, the library FRC teams have been using since 2010 without any problems!
### Numpy and OpenCV Installation
Now that we have installed robotpy, we need to install [OpenCV](https://pypi.org/project/opencv-python/) and [numpy](https://numpy.org/install/). The following terminal command will install OpenCV:
`python -m pip install opencv-python`
The following terminal command will install numpy **(using pip)**:
`python -m pip install numpy`
The following terminal command will install numpy **(using anaconda)**:
`conda install numpy`
## Contribution Workflow
There are 5 easy steps to contributing!
1. Find an issue to fix
2. Clone the repository to your local machine using [Github Desktop](https://docs.github.com/en/desktop/installing-and-authenticating-to-github-desktop/installing-github-desktop)
3. [Create a branch](https://docs.github.com/en/get-started/quickstart/github-flow#create-a-branch) of the repository to [make your commits](https://docs.github.com/en/get-started/quickstart/github-flow#make-changes) to
4. [Create a pull request](https://docs.github.com/en/get-started/quickstart/github-flow#create-a-pull-request) to merge with the main branch and have your code [reviewed](https://docs.github.com/en/get-started/quickstart/github-flow#address-review-comments) by others
5. [Delete your branch](https://docs.github.com/en/get-started/quickstart/github-flow#delete-your-branch). We don't want dead limbs on our repository tree

Here is the link to a nice tool to visualize how the branch structure works:
https://learngitbranching.js.org/

> When choosing an issue, make sure to read the tags and try to fufill the critical and milestone tasks first!

## Contribution Guidelines
To ensure quality of code, we have a set of guidelines for adding new examples to the project. These guidelines are not strictly enforced, but we do ask that you follow them when submitting pull requests with new examples. This will make the review process easier for everyone involved.

### Testing and Formatting:
 - New code must run, either on the robot or the simulator provided by RobotPy
 - Format your code with [black](https://black.readthedocs.io/en/latest/) (Note: Not required on pull-request drafts and branches in times of stress but, all code must be formatted before being merged into the main branch)

### General:
 - Every variable not calculated by the robot must be within a [Network Table v4](https://robotpy.readthedocs.io/projects/pyntcore/en/stable/ntcore.html) value
 - All code is stored in the `src` folder
 - We put all of our code into `robot.py` unless you are running a vision file or any code that needs to be in a different file
 - We use the [Command Framework](https://robotpy.readthedocs.io/en/stable/frameworks/command.html)
 - We always comment our code so that if a person from a different team sector was reading it, they would be able to read it
 - `Main.py` is never needed, it is the same as `wpilib.run(MyRobot)`
 - `robot.py` must have `#!/usr/bin/env python3` at the beginning of the file until the end of time (or python releases version 4)
  - Include the license at the beginning of every file except `robot.py`. The file `robot.py` must start with the line `#!/usr/bin/env python3` followed by the license!
 -  Don't ever check in files for your IDE (.vscode, .idea, etc)
 -  Filenames should always be all lowercase
 -   Function names are camelCase
 -   Class names start with a capital letter
 -   Class method names are camelCase
 -   Class member variables such as  `m_name`  should be  `self.name`  in Python
 -   Protected/private methods/members can optionally be prefixed with  `_`
 -   The file `robot.py` is lowercase

### MyRobot Class Structure:
#### The robotInit(self) function must be structured this way:
```python
class MyRobot(wpilib.TimedRobot):  
  def robotInit(self):  
    """  
    This function is called upon program startup and  
    should be used for any initialization code.  
    """
    # --- Define NetworkTables ---
    # --- Define Drive System ---
    # --- Define Acuators and Manipulators ---
    # --- Define Sensors ---
    # --- Define Drive Kinematics and Odometry ---
    # --- Define Vision ---
    # --- Define Acuator and Manipulator PID ---
    # --- Define Other Variables ---
    # --- Define LEDs and Pretty Stuff ---
    # --- Update NetworkTables ---
  ...
```
#### The teleopInit(self) function must be formatted this way
```python
class MyRobot(wpilib.TimedRobot):
  ...
  def teleopInit(self):
    """This function is called once each time the robot enters teleoperated mode."""
    # --- Set Local Variables ---
    # --- Reset Local Timer ---
    # --- Reset Sensors ---
    # --- Update NetworkTables ---
  ...
```
#### The teleopPeriodic(self) function must be formatted this way
```python
class MyRobot(wpilib.TimedRobot):
  ...
  def teleopPeriodic(self):
    """This function is called periodically during teleoperated mode."""
    # --- Update Variables ---
    # --- Read Sensors ---
    # --- Update Motor Values ---
    # --- Update Vital Systems ---
    # --- Update Other Systems ---
    # --- Report Telemetry to NetworkTables ---
  ...
```
#### The autonomousInit(self) function must be formatted this way
```python
class MyRobot(wpilib.TimedRobot):
  ...
  def autonomousInit(self):
    """This function is run once each time the robot enters autonomous mode."""
    # --- Set Local Variables ---
    # --- Update NetworkTables ---
    # --- Reset Local Timer ---
    # --- Reset Sensors ---
    # --- Update NetworkTables ---
  ...
```
#### The autonomousPeriodic(self) function must be formatted this way
```python
class MyRobot(wpilib.TimedRobot):
  ...
  def autonomousPeriodic(self):
    """This function is called periodically during autonomous."""
    # --- Update Variables ---
    # --- Read Sensors ---
    # --- Update Motor Values ---
    # --- Update Vital Systems ---
    # --- Update Other Systems ---
    # --- Report Telemetry to NetworkTables ---
  ...
```
