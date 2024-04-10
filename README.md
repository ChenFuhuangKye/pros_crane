# pros_crane

## Automating ROS Environment Setup
To ensure your ROS environment is automatically set up every time you open a new terminal session, you can add the setup.bash script to your .bashrc file. This will source the ROS environment setup script at each terminal launch, streamlining your workflow.

Step-by-Step Guide:
1. Open your terminal.

2. Use the following command to append the source command to your .bashrc file:

    ```
    echo "source ~/pros_crane/install/setup.bash" >> ~/.bashrc
    ```
    Replace ~/pros_crane/install/setup.bash with the actual path to your setup.bash file.

3. To apply the changes immediately, you can source your .bashrc file by running:

    ```
    source ~/.bashrc
    ```

Now, every new terminal session will automatically set up your ROS environment, saving you the step of manually sourcing setup.bash.