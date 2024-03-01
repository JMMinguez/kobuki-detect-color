# 2024-P5-FollowBall

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
