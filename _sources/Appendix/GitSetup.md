
# 🔧 Git Repo Setup


## Purpose
Setup GIT repositories and access on the **Master**.

```{note}
Don’t worry if it doesn’t work right. If everything did, you’d be out of a job.
```

(CreateRepo)=
### Create a Repository within the GitHub Classroom

1. If you don’t already have a <a href="https://github.com/" target="_blank">GitHub</a> account, create one. It’s best if your username is something identifiable (e.g., `stanbaek`).  

1. Once your account is ready, go to the <a href="https://classroom.github.com/a/gHikuWHu" target="_blank">ECE387 Classroom</a>.  

1. Your afacademy email address has been preloaded to make joining easy. While I prefer your school email address, you may use another email if desired. For this, click `Skip to the next step`:  

    ```{image} ./figures/GitClassroom_SelectIdentifier.png  
    :width: 480  
    :align: center  
    ```  
    <br>  

1. Select `Accept this assignment`.  

1. Navigate to your repository and note the repository URL. Save this link—it’s the easiest way to check if your repository is updated.  

1. Go to `Settings` and change your repository name to `ece387-lastname` (e.g., `ece387-baek`).  

    ```{important}  
    Please name your repository as `ece387-lastname` (all lowercase). This makes it easier for instructors to locate your repository.  
    ```  

## Set Up GitHub SSH Key on Master  

This section assumes you already have a GitHub account.  

1. Create an SSH key for your GitHub account by running the following command. Use the same email as your GitHub login:  

    ```bash  
    $ cd  
    $ ssh-keygen -t ed25519 -C "your_email@email.com"  
    ```  

1. When prompted to specify a file path, press `Enter` to accept the default location (`~/.ssh/id_ed25519`).  

1. Start the ssh-agent in the background and add your SSH private key:  

    ```bash  
    $ eval "$(ssh-agent -s)"  
    $ ssh-add ~/.ssh/id_ed25519  
    ```  

1. Open the public key using your favorite terminal-based text editor. This step is easier via an SSH connection from a GUI-based desktop machine, as it allows you to copy the public key to your GitHub account:  

    ```bash  
    $ nano ~/.ssh/id_ed25519.pub  
    ```  

1. Copy the entire contents of the file. Maximize the window to ensure you don’t miss anything, including the GitHub email.  

1. Open a web browser and log in to your GitHub account.  

1. In the top-right corner of any page, click your profile photo, then select **Settings**:  

    ```{image} ./figures/ssh1.png  
    :width: 180  
    :align: center  
    ```  
    <br>  

1. In the settings sidebar, click **SSH and GPG keys**:  

    ```{image} ./figures/ssh2.png  
    :width: 180  
    :align: center  
    ```  
    <br>  

1. Click **New SSH key**:  

    ```{image} ./figures/ssh3.png  
    :width: 600  
    :align: center  
    ```  
    <br>  

1. In the `Title` field, provide a descriptive label for the key, such as `master0`.  

1. Paste the copied key (from the `.pub` file) into the `Key` field.  

1. Click **Add SSH key** to save it.  

(CloneRepo)=
## Clone Repository to Your Master  

1. On the **Master**, open your GitHub repository and copy the repository address using the **SSH** mode:  

    ```{image} ./figures/GitClone.png  
    :width: 600  
    :align: center  
    ```  
    <br>  

1. Open a terminal and create a workspace source folder:  
    ```bash  
    $ mkdir -p ~/master_ws/src/  
    $ cd ~/master_ws/src  
    ```  

1. Clone your repository:  
    ```bash  
    $ git clone git@github.com:ECE387/ece387_lastname.git  
    ```  

1. Update your Git email address and name:  
    ```bash  
    $ git config user.email "you@example.com"  
    $ git config user.name "FirstName LastName"  
    ```  

1. Move into your repository you just cloned:  
    ```bash  
    $ cd ece387-lastname  
    ```  

1. Use the `touch` command to create an empty file called `COLCON_IGNORE`. This file ensures that this directory will be ignored when compiling ROS packages.

1. Run the following command to append your full name and section to a file named `README.md`. Since the file does not exist, it will be created automatically:
    ```bash  
    $ echo "# Your full name, Section" >> README.md  
    ```  

1. Verify that the file was created correctly by running:  
    ```bash  
    $ cat README.md  
    ```  
    If the file was created successfully, you should see output similar to the following:
    ```{image} ./figures/GitCreateReadmeFile.png  
    :width: 300  
    :align: center  
    ```  
    <br>  
1. Commit and push your changes to your GitHub repository:
    ```bash  
    $ git add -A 
    $ git commit -m"initial commit"
    $ git push  
    ```  


(UpstreamRepo)=
## Setup Upstream Repository  

1. Open a Terminal and navigate to your local repository, such as 
    ```bash
    $ cd  ~/master_ws/src/ece387_lastname
    ```

1. Ensure your local repository is up to date with the remote GitHub repository by running
    
    ```bash
    $ git pull 
    ```

1. Verify that all your local changes have been committed and pushed to the remote GitHub repository by running
    ```bash
    $ git status 
    ```
    If your repository is clean, you should see the following message:
    ```bash
    On branch main
    No commits yet
    nothing to commit (create/copy files and use "git add" to track)
    ```
    If you see uncommitted changes, make sure to commit and push them to the remote repository before proceeding.

1. Check your current remote repositories by typing
    ```bash
    git remote -v
    ```
    You should see two lines showing that `origin` points to your remote repository on GitHub for both fetching and pushing. 

1. Add the instructor's repository as additional remote source by running

    ```bash
    $ git remote add upstream https://github.com/ECE-387/labs.git
    $ git config pull.rebase true
    ```
1. Verify that the upstream repository has been added successfully by typing 
    ``` bash
    git remote -v
    ``` 
    You should now see two additional lines indicating `upstream` is the original repository you forked from.

    ```{image} ./figures/GitAddUpstream.gif
    :width: 640
    :align: center
    ```
    <br>

1. 
1. If the instructor updates the code, you will be notified. To get the latest updates from the upstream repository, run
    ```bash
    git pull upstream main
    ``` 

1. By default, when you push or pull your code, `origin` will be used, which points to your own GitHub repository.
    ```{image} ./figures/FetchUpstream.png
    :width: 320
    :align: center
    ```
    <br>

    <center>
    Image is sourced from <a href="https://stackoverflow.com/questions/9257533/what-is-the-difference-between-origin-and-upstream-on-github/9257901#9257901" target="_blank">Stack Overflow</a>
    </center>


<!--
## Create a repo within the GitHub Classroom:

1. Browse to [github.com](https://www.github.com) and create a GitHub account if you do not already have one. It is useful if your username is something that identifies you (e.g., bneff1013).
1. **One student per group** do the following on your personal computer:
    1. First we are going to setup the repo for the Master.  This will allow your instructor to see all of your commits throughout the semester.
    1. Browse to [https://classroom.github.com/a/OoH0u_XW](https://classroom.github.com/a/OoH0u_XW)
    1. Select "Accept this assignment"
    1. You may need to hit refresh, but eventually it will provide you a link to the repository.
	1. Browse to your repository.
	1. Note the url for your repository (save this link, it is the best way to check if your repo is updated).
	1. Go to Settings -> Manage access -> and "Invite teams or people".
	1. Provide access to your team member using their GitHub user name.
    1. Now we need to do the exact same thing to setup the repo for the robot.
	1. Browse to [https://classroom.github.com/a/0MPq4TNE](https://classroom.github.com/a/0MPq4TNE) and repeat steps c-h.
-->

<!--
## Enable SSH connection to your GitHub account
1. Open a terminal on your **Master** (ctrl+alt+t).
1. The same student as step 1.1.2 do the following:
    1. Generate a new SSH key, substituting your GitHub email address:
        ```bash
        ssh-keygen -t ed25519 -C "your_email@example.com"
        ```
	1. When you're prompted to "Enter a file in which to save the key," click enter.
	1. At the prompt, type a secure passphrase.
	1. Start the ssh-agent in the background and add your SSH private key to the ssh-agent:
        ```bash
        eval "$(ssh-agent -s)"
        ssh-add ~/.ssh/id_ed25519
        ```
	1. Open the public key:
        ```bash
        nano ~/.ssh/id_ed25519.pub
        ```
	1. Select the contents of the file (maximize the window and ensure it has your GIT email at the end), right click, and select copy.
    1. Open a web browser and sign in to your GitHub account.
	1. In the upper-right corner of any page, click your profile photo, then click **Settings**.
        ![logo](figures/userbar-account-settings.png)
	1. In the user settings sidebar, click **SSH and GPG keys**.
		![logo](figures/settings-sidebar-ssh-keys.png)
    1. Click **New SSH key**
		![logo](figures/ssh-add-ssh-key.png)
	1. In the "Title" field, add a descriptive label for the new key, such as "MasterX".
	1. Paste your key into the "Key" field (contents of the `.pub` file).
	1. Click **Add SSH key**.
	1. If prompted, confirm your GitHub password.
    1. Create a secure shell connection to your **Robot** (password is dfec3141)
        ```bash
        ssh pi@robotX
        ```
    1. Repeat steps a-f on your **Robot** and j-n on your **Master**.
-->

<!--
## Clone repository to your robot.

#####
1. Open the public key using your favorite terminal-based text editor. This step is easier via an SSH connection from a GUI-based desktop machine, as it allows you to copy the public key to your GitHub account:  
#####

1. Create a secure shell connection to your robot:
    ```bash
    ssh pi@robotX
    ```
1. Ensure you are in the ROS robot workspace src directory.
    ```bash
    cd robot_ws/src
    ```
1. Clone the robot repository:
    ```bash
    git clone git@github.com:ECE387/ece387_robot_spring202X-USERNAME.git
    ```
1. Update your git email address and the last name for you and your team mate.
    ```bash
    git config --global user.email "you@example.com"
    git config --global user.name "Lastname1 Lastname2"
    ```
-->