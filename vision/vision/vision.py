import cv2
import cv2.aruco as aruco
import numpy as np
from ultralytics import YOLO
import yaml
import argparse
from ament_index_python.packages import get_package_share_directory
import os

share_dir   = get_package_share_directory('vision')
calib_path  = os.path.join(share_dir, 'camara_calibracion.npz')
modelo_path  = os.path.join(share_dir, 'best.pt')


with np.load(calib_path) as data:
    camera_matrix = data["camera_matrix"]
    dist_coeffs = data["dist_coeffs"]

def describirEscena(input : str, arucoX : float, arucoY : float, arucoZ : float, arucoYaw : float, arucoPitch : float, arucoRoll : float, output : str):
    # Tamaño del marcador (metros)
    longitudMarcador = 0.05

    # Esquinas del marcador
    objectPoints = np.array([
        [-longitudMarcador/2,  longitudMarcador/2, 0],
        [ longitudMarcador/2,  longitudMarcador/2, 0],
        [ longitudMarcador/2, -longitudMarcador/2, 0],
        [-longitudMarcador/2, -longitudMarcador/2, 0]
    ], dtype=np.float32)

    # Cargar el modelo
    model = YOLO(modelo_path)
    # image_path = "Vision.v1i.yolov8/test/images/WIN_20250717_12_28_09_Pro_jpg.rf.9edbf5a4af5e7fb3dbb027d9c79ff507.jpg"
    # image_path = "Vision.v1i.yolov8/test/images/WIN_20250717_12_32_40_Pro_jpg.rf.ce0658ec63c1e11d0c2193ecf5ccc191.jpg"
    # image_path = "Vision.v1i.yolov8/test/images/WIN_20250717_12_39_42_Pro_jpg.rf.c66a58d1b14e9b30f7d1136c07a42817.jpg"

    # Capturar imagen desde la cámara
    # cap = cv2.VideoCapture(0)

    # if not cap.isOpened():
    #     raise RuntimeError("No se pudo abrir la cámara.")

    # print("Pulsa 'r' para capturar la imagen.")

    # while True:
    #     ret, frame = cap.read()
    #     if not ret:
    #         raise RuntimeError("Error al capturar el frame de la cámara.")
        
    #     # Mostrar el frame en una ventana
    #     cv2.imshow("Cámara", frame)

    #     # Esperar a que el usuario pulse una tecla
    #     key = cv2.waitKey(1) & 0xFF

    #     if key == ord('r'):
    #         print("Imagen capturada.")
    #         cap.release()
    #         cv2.destroyAllWindows()
    #         img = frame  # Devuelve la imagen capturada
    
    if not cv2.os.path.exists(input):
        raise FileNotFoundError(f"La imagen {input} no existe.")
    
    img = cv2.imread(input)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Para hacer la detección más robusta

    # Cargar diccionario ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()

    # Detectar marcadores
    detector = aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(gray)
    corner = corners[0][0]

    if ids is None:
        raise TypeError("Nigún marcador detectado")

    aruco.drawDetectedMarkers(img, corners, ids)
    print(f"Marcadores detectados: {ids.flatten()}")

    retval, rvec, tvec = cv2.solvePnP(objectPoints, corner, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)

    if retval:
        cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, longitudMarcador / 2)

    # 3) Define las coordenadas reales de esas esquinas en el plano del marcador

    # Fíjate que el origen (0,0) es el centro del ArUco,
    # y X+ va hacia la derecha, Y+ hacia arriba
    world = np.array([
        [-longitudMarcador / 2, longitudMarcador / 2],  # esquina superior-izq
        [longitudMarcador / 2, longitudMarcador / 2],  # esquina superior-der
        [longitudMarcador / 2, -longitudMarcador / 2],  # esquina inferior-der
        [-longitudMarcador / 2, -longitudMarcador / 2],  # esquina inferior-izq
    ], dtype=np.float32)

    # 4) Calcula la homografía que lleva píxeles → metros
    #    getPerspectiveTransform espera src=imagen, dst=mundo
    H = cv2.getPerspectiveTransform(corner, world)

    # Run batched inference on a list of images
    result = model(input)  # return a list of Results objects
    result = result[0]

    # print(result.boxes.xywh)

    # Preparo la escritura del fichero .yaml
    arucoInfo = {
        "posicion": {"x": arucoX, "y": arucoY, "z": arucoZ},
        "orientacion": {"yaw": arucoYaw, "pitch": arucoPitch, "roll": arucoRoll}
    } # Contiene información sobre la posición del marcador ArUco respecto del frame world
    objetos = [] # Recoge información de los objetos de la escena

    for i, caja in enumerate(result.boxes.xywh):
        x_center, y_center, width, height = caja
        class_id = int(result.boxes.cls[i].item())
        confidence = result.boxes.conf[i].item()
        class_name = model.names[class_id]  # Nombre de la clase (si está disponible)

        # Convertir coordenadas a enteros
        x_center, y_center = int(x_center), int(y_center)

        # Dibujar el centroide
        cv2.circle(img, (x_center, y_center), 5, (0, 0, 255), -1)  # círculo rojo
        # Dibujar etiqueta con clase y confianza
        cv2.putText(img, f"{class_name} {confidence:.2f}", (x_center + 10, y_center - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        print(f"Objeto {i}: {class_name} | Confianza: {confidence:.2f} | Centroide: ({x_center}, {y_center})")

        # 6) Transfórmalo con cv2.perspectiveTransform
        pt_px = np.array([[[x_center, y_center]]], dtype=np.float32)  # shape (1,1,2)
        pt_m = cv2.perspectiveTransform(pt_px, H)[0, 0]  # shape (2,)

        X_m, Y_m = pt_m
        print(f"Cilindro en sistema ArUco: X={X_m:.3f} m, Y={Y_m:.3f} m")
        objeto = {
            "id": class_id,
            "posicion": {"x": float(X_m), "y": float(Y_m), "z": 0.0},
            "orientacion": {"yaw": arucoYaw, "pitch": arucoPitch, "roll": arucoRoll}
        }
        objetos.append(objeto)

    datos = {
        "aruco": arucoInfo,
        "objetos": objetos
    }

    with open(output, "w") as f:
        yaml.dump(datos, f, default_flow_style=False)

    if ids is not None:
        src_pts = corners[0].reshape(4, 2).astype(np.float32)

        # 2) Define el tamaño de salida en píxeles
        out_size = 640
        dst_pts = np.array([
            [0, 0],  # esquina sup‑izq
            [out_size - 1, 0],  # sup‑der
            [out_size - 1, out_size - 1],  # inf‑der
            [0, out_size - 1]  # inf‑izq
        ], dtype=np.float32)

        # 3) Homografía píxel→píxel
        H_img = cv2.getPerspectiveTransform(src_pts, dst_pts)

        h, w = img.shape[:2]
        img_corns = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float32).reshape(-1, 1, 2)
        warped_corns = cv2.perspectiveTransform(img_corns, H_img).reshape(-1, 2)

        x_min, y_min = warped_corns.min(axis=0)
        x_max, y_max = warped_corns.max(axis=0)
        W = int(x_max - x_min)
        H = int(y_max - y_min)

        T = np.array([[1, 0, -x_min],
                      [0, 1, -y_min],
                      [0, 0, 1]])
        H_full = T @ H_img  # primero H_img, luego traslación

        full_rectified = cv2.warpPerspective(img, H_full, (W, H))

        cv2.namedWindow("Mesa completa rectificada", cv2.WINDOW_NORMAL)
        cv2.imshow("Mesa completa rectificada", full_rectified)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    else:
        print("No se detectaron marcadores")

    # Mostrar la imagen
    cv2.imshow("Centroides detectados", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="Generar YAML con ArUco y objetos.")
    parser.add_argument("--input", type=str, required=True, help="Ruta de la imagen de entrada")
    parser.add_argument("--x", type=float, default=0.445, help="Posición X del ArUco")
    parser.add_argument("--y", type=float, default=-0.345, help="Posición Y del ArUco")
    parser.add_argument("--z", type=float, default=0.0, help="Posición Z del ArUco")
    parser.add_argument("--yaw", type=float, default=3.1415, help="Orientación yaw del ArUco (radianes)")
    parser.add_argument("--pitch", type=float, default=0.0, help="Orientación pitch del ArUco (radianes)")
    parser.add_argument("--roll", type=float, default=0.0, help="Orientación roll del ArUco (radianes)")
    parser.add_argument("--output", type=str, default=os.path.join(share_dir, 'objetos.yaml'), help="Nombre del archivo de salida")

    args = parser.parse_args()
    describirEscena(args.input, args.x, args.y, args.z, args.yaw, args.pitch, args.roll, args.output)

if __name__ == "__main__":
    main()
