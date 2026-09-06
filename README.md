# Swarm UAVs project

## Hardware requirements:

Ubuntu 22.04 with minimum 16GB RAM and 60GB available ROM, and external GPU (optional)

ROS2-Humble and python 3.10

## Setups:

### 0. [Miniconda](https://docs.anaconda.com/free/miniconda/miniconda-install/)

```
bash cmd/setup_miniconda.sh
```
### 1. Install conda environment (uav)

```
conda env create -f environment.yml
conda activate uav
#pip install mavsdk asyncio --force
```
### 2. Gazebo ROS2:

To install [Gazebo Harmonic (Gazebo Sim)](https://gazebosim.org/docs/harmonic/install_ubuntu/)

Follow this instruction to install ROS (Optional): [Install ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)</br>

### 3. [PX4-Autopilot](https://github.com/PX4/PX4-Autopilot.git)

```bash
bash cmd/setup_px4.sh
```

#### Gazebo battery and motor-power extension

`setup_px4.sh` tự cài `MotorPowerSystem`, nối dữ liệu pin vào `GZBridge` và
build PX4. Thông số pin được chỉnh trong `config/uav_config.yaml` tại hai mục
`gazebo_battery_model` và `px4_parameters`; không cần sửa source PX4.
Chi tiết: [ENERGY_SIMULATION.md](px4_extensions/battery_power/ENERGY_SIMULATION.md).

Sau khi đổi cấu hình, chạy lại `./swam_uav.sh` để tự đồng bộ và build. Kiểm tra
dữ liệu bằng:

```bash
gz topic -e -n 1 -t /model/x500/motor_power
gz topic -e -n 1 -t /model/x500/battery/linear_battery/state
```

### 7. [QGroundControl Ground Control Station](https://github.com/mavlink/qgroundcontrol/releases) (Optional)

## Cài đặt và chạy bằng Docker

Docker image đã bao gồm Ubuntu 22.04, Gazebo Harmonic, công cụ build PX4,
Python và các thư viện của ứng dụng. Mã nguồn trên host được mount vào `/app`
trong container nên kết quả build PX4 và thay đổi source vẫn được lưu trong
repository.

### 1. Cài Docker Engine (chỉ thực hiện một lần)

Từ thư mục gốc của repository, chạy script bằng user hiện tại, không thêm
`sudo` trước script:

```bash
cd ~/workspace/SW_UAV
bash cmd/install_docker.sh
```

Script cài Docker Engine, Docker Compose, Buildx và thêm user hiện tại vào
group `docker`. Sau khi cài xong, đăng xuất rồi đăng nhập lại hoặc chạy:

```bash
newgrp docker
```

Kiểm tra cài đặt và quyền truy cập Docker:

```bash
id -nG
docker --version
docker compose version
docker run --rm hello-world
```

`id -nG` phải có group `docker`. Nếu gặp lỗi truy cập
`/var/run/docker.sock`, hãy mở terminal mới sau khi đăng nhập lại; không sửa
socket bằng `chmod 666`.

### 2. Build image và khởi động container (CPU/AMD)

Từ thư mục `docker`, dùng một lệnh để build image và chạy container:

```bash
cd ~/workspace/SW_UAV/docker
docker compose up -d --build
```

Lệnh trên tạo image `sw-uav:latest`, tạo container `sw-uav` và chạy container
ở chế độ nền. Cấu hình Compose mặc định sử dụng PyTorch CPU, phù hợp với máy
CPU-only hoặc GPU AMD.

```bash
docker compose ps
docker exec sw-uav python -c "import sklearn; print(sklearn.__version__)"
```

Ở các lần chạy sau, nếu Dockerfile và requirements không thay đổi, chỉ cần:

```bash
cd ~/workspace/SW_UAV/docker
docker compose up -d
```

Không chạy `docker compose pull`: image của dự án được build từ Dockerfile
trên máy và không được tải từ registry. Muốn cập nhật base image Ubuntu, dùng
`docker compose build --pull` rồi chạy `docker compose up -d`.

### 3. Build và chạy PX4 SITL

Không cần mở shell bên trong container. Chạy PX4 trực tiếp từ terminal host:

```bash
docker exec -it sw-uav zsh
```

Lần đầu PX4 sẽ được build trước khi PX4 SITL và Gazebo khởi động. Lệnh giữ
terminal hiện tại để hiển thị log. Nhấn `Ctrl+C` để dừng.

### 4. Chạy giao diện hoặc toàn bộ mô phỏng

Nếu giao diện Qt/Gazebo bị từ chối truy cập màn hình, chạy trên host:

```bash
xhost +local:docker
```

Chạy riêng giao diện trong container từ một terminal host khác:

```bash
docker exec -it sw-uav bash -lc 'cd /app && python src/app.py'
```

Hoặc chạy toàn bộ mô phỏng bằng script điều phối trên host:

```bash
cd ~/workspace/SW_UAV
./swam_uav.sh
```

Không chạy đồng thời lệnh PX4 thủ công ở bước 3 và `swam_uav.sh`, vì script
điều phối sẽ tự chuẩn bị và khởi động các tiến trình PX4.

### 5. NVIDIA CUDA (tùy chọn)

Docker Compose ở bước 2 mặc định dùng CPU. Trên máy NVIDIA, script điều phối
sẽ tự kiểm tra GPU, CUDA và NVIDIA Container Toolkit:

```bash
cd ~/workspace/SW_UAV
./docker/build.sh --detect
./swam_uav.sh
```

Script chọn `cu118`, `cu124`, `cu126` hoặc tự chuyển về CPU. Host NVIDIA cần
driver và NVIDIA Container Toolkit; không cần cài CUDA Toolkit hoặc cuDNN.

### 6. Rebuild hoặc cài lại image

Sau khi thay đổi Dockerfile hoặc requirements:

```bash
cd ~/workspace/SW_UAV/docker
docker compose down
docker compose up -d --build
```

Chỉ rebuild không cache khi nghi ngờ cache cũ bị lỗi:

```bash
cd ~/workspace/SW_UAV/docker
docker compose down
docker compose build --no-cache
docker compose up -d --no-build
```

Cách rebuild không cache mất nhiều thời gian và tải lại toàn bộ dependencies.

Các lệnh quản lý thường dùng:

```bash
docker compose ps       # xem trạng thái
docker compose logs -f  # theo dõi log
docker compose stop     # dừng nhưng giữ container
docker compose down     # dừng và xóa container
```


## Run program without Docker

### 1. Run all:
Terminal 1
```
./swam_uav.sh 
```
Terminal 2
```
conda activate uav
python src/main.py
```

### 2. Run only UI

```
python src/app.py
```

```
python src/interface_base.py
```

```
python src/interface_map.py

```
## Debug

1. Check opening ports
TCP

```

    netstat -ltnp

```

UDP

```

    netstat -lunp

```

UARTs

```

     ls /dev/tty*

````
2. Debug programs
```Interface
   gdb --agrs python src/app.py
````

## Collaborators:
