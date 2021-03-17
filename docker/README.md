# Docker Workflow

## Updating Dependencies for Travis

When new dependencies are required and they should be added to the CI as well, the following workflow is recommended:
- Add the dependencies to the install_deps.sh
- Use the Dockerfile.base to create a docker image containing all the dependencies installed except for the OpenREALM library
  ```
  # Execute for image building
  docker build -t openrealm-base:latest - < Dockerfile.base
  ```
- Tag it with your username and upload it to dockerhub
  ```
  docker tag openrealm-base:latest laxnpander/openrealm-base:latest
  docker push openrealm-base:latest
  ```
- Now that the docker image is in the cloud, it can be accessed by Travis for CI. Modify the .travis file to point to the correct dependency image:
  ```
  before_install:
    - docker pull laxnpander/openrealm-base:latest
    - docker run -it -d --name build laxnpander/openrealm-base:latest bash
  ```
Note: This workflow is only reasonable, when the dependencies don't change frequently. Due to the fact, that travis is using a docker as initial setup testing
times are drastically reduced. However, when the install_deps.sh and the uploaded docker image are different, potential errors will not be covered by travis as
it may be using an old/outdated image.