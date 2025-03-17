# Lab 4: Kamera og Kamerasyn
I denne laboppgaven skal dere sette opp et kamerasystem og datasynalgoritmer, slik at dere kan bruke kameraer i ROS2

Dette er en oppgave for to personer, hvor dere gjør to forskjellige oppgaver som til sammen blir en helhet. Den ene lager en konfigurasjons fil for å starte opp mange prosesser (oppgaver merket med A) og den andre lager URDF (oppgaver merket med B). På grunn av hvordan ROS2 er oppbygd skal PCene deres kunne kommunisere med hverandre hvis dere er på samme nettverk. Merk at dere burde jobbe for at begge blir ferdige med oppgave 1 før dere begynner på oppgave 2, osv. Fordi det er enklere å sjekke at nodene fungerer når de blir testet sammen.
# Oppgave 1
## Del A
Du kan kun gjøre del A hvis du har en Linux-PC. Har du en Windows-PC kan du gjøre del B. 

Finn ett USB-kamera på laben, eller bruk et som er integrert på datamaskinen din. Kjør
```
v4l2-ctl --list-devices
```
Du vil se en liste over kameraer som er tilgjengelig på maskinen din. Det kommer på formen "dev/videoX". Det vil muligens være flere "/dev/videoX" per kamera. Oftest er da den med lavest nummer riktig, men prøv de med høyere nummer hvis ikke.

Kjør
```
sudo apt install ros-jazzy-usb-cam
```
for å sikre at `usb_cam`-pakken er installert.
Kjør så
```
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/videoX
```
Hvor du bytter ut videoX med det tallet du fant tidligere.

Kjør så `rqt` og bruk Image Plot til å se at kameraet funker.
## Del B 
Du kan finne et bilde og plassere det en plass på datamaskinen. Du skal deretter starte noden `image_publisher` i pakken `image_publisher`. 
```
ros2 run image_publisher image_publisher /sti/til/bildefil.png
```
Hvis du nå kjører `rqt` og åpne Image Plot for å finne bildet. 
# Oppgave 2
## Del A
Det neste du skal gjøre er å kalibrere kameraet ditt. Følg denne tutorialen [her](https://docs.ros.org/en/ros2_packages/rolling/api/camera_calibration/doc/tutorial_mono.html) for å kalibrere kameraet. Når du er ferdig skal du ha en kalibreringsfil med YAML filtype.
## Del B
Du skal nå lage en ny pakke. Du bestemmer selv om den skal være C++ eller Python. Kall den `camera_pipeline`

Det første du skal gjøre er å lage en launch-mappe med en launch-fil som heter `pipeline.launch.py` eller noe lignende. Launch-filen skal starte opp en node som heter "rectify_node" fra "image_proc" pakken.

Husk å endre på setup.py/CMakeList.txt slik at du flytter launch-filen til `share`.
## Del A og B
Hvis person A nå kjører kameranoden, vil du forhåpentligvis kunne få en `image_raw` og `camera_info` topic, som sender henholdsvis bilde og kalibreringsparameter.

Når person B nå kjører sin launch-fil, så vil forhåpentligvis det dukke opp et kalibrert bilde på `/image_rect`.

# Oppgave 3
## Del A
Lag en ny pakke, enten Python eller C++, som du kaller `camera_pipeline`. Lag en node du kaller `gaussian_blur`.

Nedenfor finner du en kode du kan bruke for noden
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2

class GaussianBlurNode(Node):
    """
    A node for blurring an image
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the Gaussian blur node.
        """
        super().__init__('gaussian_blur')

        # Subscribe to a image topic
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the filtered image
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
  
        # Initialize CVBridge
        self.bridge = CvBridge()
    

    def image_callback(self, msg):
        """
        Callback function for input image topic.
        Applies Gaussian blur to the received image and publishes the edges as an image.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        # 
        # Legg koden din her
        # cv_image er bildet du skal bruke som input
        # cv_blurred (en variabel du lager selv) skal være sluttresultatet etter å bruke gaussisk uskarphet.
        
        # Convert back to ROS Image message
        try:
            blur_msg = self.bridge.cv2_to_imgmsg(cv_blurred, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
  
        # Publish the filtered image
        self.publisher.publish(blur_msg)

# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    image_blur_node = GaussianBlurNode()
    rclpy.spin(image_blur_node)
    image_blur_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
I koden over ser du en subscriber som mottar en bildemelding fra `image_raw`. Den konverterer meldingen til et OpenCV-bilde, som heter `cv_image` via `cv_bridge`-biblioteket. Etter det skal du bruke gaussisk uskarphetsalgoritmen med en 5x5 kjerne. Resultatet setter du i variabelen `cv_blurred`. Den blir igjen konvertert til en melding som publiseres til `output_image`.

Du må selv bruke OpenCV til å lage det uskarpe bildet.
## Del B
I pakken din `camera_pipeline` lager du en node du kaller `canny_edge`.

Nedenfor finner du en kode du kan bruke for noden
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2

class CannyEdgeNode(Node):
    """
    A node for apply the canny edge method on an image
    """
    def __init__(self):
        """
        Constructs all the necessary attributes for the node.
        """
        super().__init__('gaussian_blur')

        # Subscribe to a image topic
        self.subscription = self.create_subscription(
            Image,
            "image_raw",
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Publish the canny edge detection image
        self.publisher = self.create_publisher(
            Image,
            'output_image',
            10)
  
        # Initialize CVBridge
        self.bridge = CvBridge()
    

    def image_callback(self, msg):
        """
        Callback function for input image topic.
        Applies the Canny Edge to the received image and publishes the edges as an image.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
        # 
        # Legg koden din her
        # cv_image er bildet du skal bruke som input
        # cv_edge (en variabel du lager selv) skal være sluttresultatet etter å bruke Canny Edge.
        
        # Convert back to ROS Image message
        try:
            edge_msg = self.bridge.cv2_to_imgmsg(cv_edge, "bgr8")
        except Exception as e:
            self.get_logger().error('Failed to convert image: %s' % str(e))
            return
  
        # Publish the Canny Edge image
        self.publisher.publish(edge_msg)

# Initialize the node
def main(args=None):
    rclpy.init(args=args)
    canny_edge_node = CannyEdgeNode()
    rclpy.spin(canny_edge_node)
    cannt_edge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
I koden over ser du en subscriber som mottar en bildemelding fra `image_raw`. Den konverterer meldingen til et OpenCV-bilde, som heter `cv_image` via `cv_bridge`-biblioteket. Etter det skal du bruke Canny Edge-algoritmen med grenser du setter selv. Resultatet setter du i variabelen `cv_edge`. Den blir igjen konvertert til en melding som publiseres til `output_image`.

Du må selv bruke OpenCV til å detektere kantene.
## Del A og B
Sammen skal dere oppdatere launch-filen fra forrige oppgave. Person A flytter sin `gausian_blur`-node til samme pakke som person B, og person B oppdaterer launch-filen slik at både `gaussian_blur`- og `canny_edge`-noden starter opp. Dere skal også bruke remapping (siden nå abonnerer begge på `image_raw` og publiserer på `image_output`). Dere skal skifte via remappe `image_raw` til `image_rect` for gaussian_blur, så den lytter til det kalibrerte bildet. `image_output` skiftes til `image_blurred`. For canny_edge skifter dere `image_raw` til `image_blurred`.

Kjører dere nå launch-filen og kameranoden vil dere få mange topics. Ser dere da på `image_output` på `rqt`, vil dere se et ferdig prosessert bilde. Ser dere på `rqt_graph`, finner dere også hele pipeline-en fra kamera til det prosesserte bildet.
