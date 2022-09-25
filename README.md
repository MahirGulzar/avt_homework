
![example workflow](https://github.com/MahirGulzar/avt_homework/actions/workflows/docker-image.yml/badge.svg)

##  Description
<br />
Traffic light fetcher ros package that detects traffic light from an input video stream and publishes related information.
<br />
<br />

## Structure

    ├── ...
    ├── src
    │   ├── traffic_light_fetcher          # ROS nodes
    │      │── nodes
    │      │   ├── traffic_light_fetcher
    │      │   │   │── tl_fetcher
    │      │   │   │── tl_analysis
    │      │...
    │      │── src                         # Modules
    │      │   ├── traffic_light_fetcher
    │      │   │   │── tl_detection.py
    

## How to run

<br />
<b>Note:</b> Docker container uses host's gpu device.
<br />
<br />

- Using ```docker-compose```
```console
docker-compose up --build
```
- Using run script (if ```docker-compose``` is not installed)
```console
cd <repo directory>
./run
```
