# Nodes

## master_node

This node is the main node that ties the other nodes together and performs the main logic of the system.
The main routine consists of
- searching for the apple
- centering on the apple
- reaching for the apple
- grapping the apple
- disposing of the apple
Once it completes its task, it then destroys itself.

# Other Functions

## ImageProcess.py

This function is used to process the image from the arm camera. It applies a red mask and identifies the location of the apple.

Hopefully this will be replaced by a service call
