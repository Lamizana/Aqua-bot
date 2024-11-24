#!/usr/bin/env python3
# --------------------------------------------------------- #
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

# --------------------------------------------------------- #
class QRCodeDetectorNode(Node):
    """
    Utiliser ROS 2 pour recevoir les images via un suscriber.
    Converti les images ROS en format OpenCV avec cv_bridge.
    Détecte et décode les QR codes dans chaque frame reçue.
    """

    def __init__(self):
        """
        Initialiser un suscriber et un publisher pour  la caméra :
        """

        super().__init__('qr_code_detector')
        self.subscriber = self.create_subscription(
            Image,
            '/aquabot/sensors/cameras/main_camera_sensor/image_raw',
            self.image_callback,
            10
        )

        # Publisher pour envoyer les données dans le topic cible :
        self.publisher = self.create_publisher(
            String,
            '/vrx/windturbinesinspection/windturbine_checkup',
            10
        )

        # Initialiser le détecteur de QR codes et le convertisseur ROS-OpenCV
        self.detector = cv2.QRCodeDetector()
        self.bridge = CvBridge()


    def image_callback(self, msg):
        """
        Callback pour traiter les images reçues.
        Détecte et décode les QR codes.
        """

        try:
            # Convertir le message ROS en image OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Détecte et décode le QR code :
            data, bbox, _ = self.detector.detectAndDecode(frame)
            id = data[6: 7]
            state = data[17: 19]

            if data:
                # Affiche les données décodées :
                self.get_logger().info(f"QR Code detected:\n- Id: {id}\n- State: {state}\n")
                # Publier les données décodées dans le topic cible :
                self.publish_data(data)

            # Dessiner la boîte englobante si détectée :
            if bbox is not None:
                bbox = bbox.astype(int)  # Convertir en entiers
                for i in range(len(bbox[0])):
                    start_point = tuple(bbox[0][i])
                    end_point = tuple(bbox[0][(i + 1) % len(bbox[0])])
                    cv2.line(frame, start_point, end_point, (0, 255, 0), 2)

            # Afficher l'image avec les annotations :
            cv2.imshow("QR Code Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Quitter avec 'q'
                self.get_logger().info("Shutting down QR code detector.")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


    def publish_data(self, data):
        """
        Publie les données décodées des QR codes dans le topic cible.
        param data: Les données décodées du QR code.
        """

        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        self.get_logger().info(f"Published data: {data}")


# --------------------------------------------------------- #
def main(args=None):

    # Initialise le système ROS 2 :
    rclpy.init(args=args)

    # Creation du Node :
    node = QRCodeDetectorNode()

    # Maintient le Node actif et en fonctionnement tant qu'il 
    # reçoit des messages ou des événements à traiter :
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped by user.")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


# --------------------------------------------------------- #
if __name__ == "__main__":
    main()
