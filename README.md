# A.RobotAI-ros2-streamer
To make and send stream of ROS 2 images captured from carmera attached to Robot.

```
연구개발과제명: 일상생활 공간에서 자율행동체의 복합작업 성공률 향상을 위한 자율행동체 엣지 AI SW 기술 개발

세부 개발 카테고리
● 지속적 지능 고도화를 위한 자율적 흐름제어 학습 프레임워크 기술 분석 및 설계
- 기밀성 데이터 활용 지능 고도화를 위한 엣지와 클라우드 분산 협업 학습 프레임워크 기술
- 엣지와 클라우드 협력 학습 간 최적 자원 활용 및 지속적 지능 배포를 위한 자율적 학습흐름제어 기술

개발 내용 
- 엣지와 클라우드 분산 협업을 위한 지속적 지능 배포 프레임워크 
- 자율행동체 엣지 기반 클러스터링 솔루션 및 분산 학습 프레임워크 개발
```

---

# ROS Streamer

This is a ROS package that streams images from a camera to zmq sockets. It use to be a federated learning perception system.

## Ros Streamer Server Node

The streamer node is a ROS node that streams images from a camera to zmq sockets.

### 1. Ros Streamer Build

```bash
# Makefile build
make build
```

### 2. Ros Streamer Launch

```bash
# Makefile launch
make launch
```

### 3. Docker Build and Push

```bash
# Makefile build
make docker-build

# Makefile build and push
make docker-push
```

### 4. Kubernetes Deploy

```bash
# Kubernetes deploy
cd src/ros_streamer_server/kube
kubectl apply -f ros-streamer-deployment.yaml
```

### 5. Kubernetes Delete
# ROS Streamer

This is a ROS package that streams images from a camera to zmq sockets. It use to be a federated learning perception system.

## Ros Streamer Server Node

The streamer node is a ROS node that streams images from a camera to zmq sockets.

### 1. Ros Streamer Build

```bash
# Makefile build
make build
```

### 2. Ros Streamer Launch

```bash
# Makefile launch
make launch
```

### 3. Docker Build and Push

```bash
# Makefile build
make docker-build

# Makefile build and push
make docker-push
```

### 4. Kubernetes Deploy

```bash
# Kubernetes deploy
cd src/ros_streamer_server/kube
kubectl apply -f ros-streamer-deployment.yaml
```

### 5. Kubernetes Delete

```bash
# Kubernetes delete
cd src/ros_streamer_server/kube
kubectl delete -f ros-streamer-deployment.yaml
```

## Ros Streamer Viewer

The viewer is a python script that subscribes to the zmq socket and displays the images.

### 1. Ros Streamer Viewer Build

```bash
pyinstaller --onefile viewer/viewer.py
```

### 2. Ros Streamer Viewer Run

```bash
# Default viewer (localhost:5555)
./dist/viewer run
```

### 3. Ros Streamer Viewer Run with Arguments

```bash
# Viewer with arguments
./dist/viewer run --host 136.166.201.33 --port 15555
# Short version
./dist/viewer run -h 136.166.201.33 -p 15555
```

```bash
# Kubernetes delete
cd src/ros_streamer_server/kube
kubectl delete -f ros-streamer-deployment.yaml
```

## Ros Streamer Viewer

The viewer is a python script that subscribes to the zmq socket and displays the images.

### 1. Ros Streamer Viewer Build

```bash
pyinstaller --onefile viewer/viewer.py
```

### 2. Ros Streamer Viewer Run

```bash
# Default viewer (localhost:5555)
./dist/viewer run
```

### 3. Ros Streamer Viewer Run with Arguments

```bash
# Viewer with arguments
./dist/viewer run --host 136.166.201.33 --port 15555
# Short version
./dist/viewer run -h 136.166.201.33 -p 15555
```
