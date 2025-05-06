# 🧠 ArUco Marker Detection with ROS (Noetic)

Este projeto ROS deteta marcadores ArUco utilizando imagens de uma câmara (ex: Raspicam). Os marcadores são desenhados na imagem e a pose é publicada em `/aruco_detections`.

---

## 📦 Estrutura do workspace

Certifica-te que tens um workspace `catkin_ws` funcional. Este pacote vai dentro da pasta `src/`.

```bash
catkin_ws/
├── src/
│   ├── tralha/
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   └── scripts/
│   │       └── detect_aruco.py
```

---

## 🔧 Pré-requisitos

- Ubuntu 20.04
- ROS Noetic
- OpenCV (`cv2`)
- `cv_bridge` e `image_transport`
- Python3

Instalar dependências (se necessário):

```bash
sudo apt update
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport python3-opencv
```

---

## 🚀 Instalação

1. Entra no teu workspace:

```bash
cd ~/catkin_ws/src
```

2. Cria o pacote:

```bash
catkin_create_pkg tralha rospy std_msgs sensor_msgs cv_bridge image_transport
```

3. Cria a pasta de scripts e adiciona o código:

```bash
mkdir -p tralha/scripts
gedit tralha/scripts/detect_aruco.py
```

4. Cola o conteúdo do script (ver abaixo).

5. Dá permissões de execução:

```bash
chmod +x tralha/scripts/detect_aruco.py
```

6. Adiciona ao `CMakeLists.txt` (no final):

```cmake
catkin_install_python(PROGRAMS
  scripts/detect_aruco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

---

## 📸 Script: detect_aruco.py

```python
#!/usr/bin/env python3

import rospy
import cv2
import cv2.aruco as aruco
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image/compressed", Image, self.image_callback)
        self.image_pub = rospy.Publisher("/aruco_detections", Image, queue_size=10)

        # Usa dicionário ArUco 6x6 250
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        rospy.loginfo("Aruco Detector Ready ✅")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("Erro a converter imagem: %s", e)
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            aruco.drawDetectedMarkers(cv_image, corners, ids)
            rospy.loginfo(f"Markers Detetados: {ids.flatten()}")

        out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.image_pub.publish(out_msg)

if __name__ == '__main__':
    try:
        ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
```

---

## 🛠 Compilar

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## ▶️ Executar

```bash
roscore
```

Noutro terminal:

```bash
rosrun tralha detect_aruco.py
```

---

## 👁️ Ver imagem com deteções

Usa o `rqt_image_view`:

```bash
rosrun rqt_image_view rqt_image_view
```

Seleciona o tópico:

```
/aruco_detections
```

---

## 🧪 Debugging

- Verifica que o tópico da câmara está ativo (ex: `/raspicam_node/image/compressed`)
- Garante que `cv_bridge` e `image_transport` estão corretamente instalados.
- Podes usar `rqt_graph` para confirmar que os nós e tópicos estão ligados corretamente.

---

## 📦 Extras

- Podes alterar o dicionário ArUco em `aruco.Dictionary_get(...)` para 4x4, 5x5, 7x7 etc.
- A pose dos marcadores pode ser calculada se tiveres a calibração da câmara e usares `cv2.aruco.estimatePoseSingleMarkers`.

---

## 🧠 Créditos

Desenvolvido com ❤️ para projetos com ROS e visão por computador.
