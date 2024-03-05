# 2024-P5-FollowBall
## Introduccion
El objetivo de la practica es conseguir que el [Kobuki](https://robots.ros.org/kobuki/) avance hacia una pelota filtrando por color y teniendo en cuenta objetos del entorno.

## Descripcion y procedimiento
Para esta practica, es necesario usar los paquetes que se encuentran dentro de la carpeta [sensors](https://github.com/Docencia-fmrico/ASR_2024/tree/main/sensors) proporcionados por [fmrico](https://github.com/fmrico). 

Es necesario modificar dischos paquetes para que generesn vectores de atraccion y repulsion usando Vector3:
En el caso de la camara:
```cpp
HSVFilterNode::HSVFilterNode()
: Node("hsv_filter_node")
{
  a_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3>("attractive_vector", 10);

  object_pub_ = create_publisher<std_msgs::msg::Bool>(
    "object", 100);
}

void
HSVFilterNode::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & image)
{
  object_msg.data = true;

  cv::Rect bbx = cv::boundingRect(image_filtered);
  if (bbx.width == 0 || bbx.height == 0) {
    RCLCPP_WARN(get_logger(), "Object not detected");
    return;
    object_msg.data = false;
  }

  cv::Point2d point = get_detected_center(image_filtered);
  attractive_vector_msg.x = point.x;
  attractive_vector_msg.y = point.y;
  attractive_vector_msg.z = 0.0;

  object_pub_->publish(object_msg);
  a_vector_pub_->publish(attractive_vector_msg);
}
```

En el caso del laser:
```cpp
ObstacleDetectorNode::ObstacleDetectorNode()
: Node("obstacle_detector_node")
{
  r_vector_pub_ = create_publisher<geometry_msgs::msg::Vector3>("repulsive_vector", 10);
}

void
ObstacleDetectorNode::laser_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan)
{
  if (distance_min < 1) {
    repulsive_vector_msg.x = -distance_min * cos(angle);
    repulsive_vector_msg.y = -distance_min * sin(angle);
    repulsive_vector_msg.z = 0.0;

  } else {
    repulsive_vector_msg.x = 0.0;
    repulsive_vector_msg.y = 0.0;
    repulsive_vector_msg.z = 0.0;
  }

  r_vector_pub_->publish(repulsive_vector_msg);
}
```

Luego en mi nodo he usado una FSM en funcion de los valores que reciba el Kobuki
![WhatsApp Image 2024-03-05 at 11 41 42](https://github.com/Docencia-fmrico/p5-followball-jmartinm2021/assets/92941332/e5b24590-59fd-486b-bb96-8c9408ec3479)


## Enunciado
Crea una aplicación en ROS 2 que haga que un robot busque y siga una pelota de un color característico, esquivando los obstáculos que se encuentre en su camino, usando VFF.

![vff1 drawio](https://github.com/Docencia-fmrico/2024-P6-FollowBall/assets/3810011/f485341e-c5b3-4515-b5c8-673b6e708632)

* Debes crear un paquete que tenga un nodo que **use** la salida de los nodos que ya existen(https://github.com/Docencia-fmrico/ASR_2024/tree/main/sensors) en ASR24, con ligeras modificaciones. No debes copiar ni ficheros ni código de ASR24 a tu paquete nuevo, solo modificarlos en su ubicación original.
* La modificación de los nodos que hay en ASR24 consiste en que cada uno publique los componentes necesarios de un vector atractivo, en el caso de `hsv_filter` o un vector repulsivo, en el caso de `ObstacleDetector`. Puedes usar `geometry_msgs/msg/Vector3` o cualquier interfaz que te parezca adecuada.
* El nodo desarrollado debe tener dos estados:
    * Uno para cuando no percibe el objeto, y gira hacia un lado. Por ejemplo, podría girar siempre hacia el mismo lado, o hacia el lado donde estaba la pelota antes de desaparecer.
    * Otro cuando el objeto es percibido, en el que genera velocidades usando VFF.
* Crea un launcher para lanzarlo todo.
* Una parameters para cualquier valor parametrizable: velocidades, valores del filtro, etc.
* Usa topics con nombre genérico y conectalo con remapeos en el launcher. Usa launchers y/o configuraciones de parámetros diferentes cuando estés en el simulador o en el robot real.

![vff drawio (2)](https://github.com/Docencia-fmrico/2024-P6-FollowBall/assets/3810011/4519e857-a055-4049-a6da-0b42efe2d787)
