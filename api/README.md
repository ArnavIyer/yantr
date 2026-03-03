This repo is for the server that the robot will interact with.
- streams images from the robot, where they will go through a pipeline to extract objects
- VLA -> SAM3 -> SigLIP2 -> storage
- The LLM can interface with this server to search for objects in the scene when it gets a relevant query.
