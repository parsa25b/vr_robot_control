# Install dependencies (if any)
sudo apt-get update \
        && sudo apt-get -y install python3-tk \
        xvfb \
        android-tools-adb

# Install oculus_reader
pip install git+https://github.com/rail-berkeley/oculus_reader.git