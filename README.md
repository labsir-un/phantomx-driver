# Introducciona ROS 


## Que es ROS 
ROS es un meta sistema operativo de código abierto para tu robots. Proporciona los servicios que presta un sistema operativo, como la abstracción de hardware, control de dispositivos de bajo nivel, implementación de funcionalidades comunes, comunicación entre procesos mediante intercambio de mensajes y gestión de paquetes. Además, ofrece herramientas y bibliotecas para obtener, construir, escribir y ejecutar código en múltiples computadoras. En algunos aspectos, ROS es similar a otros "frameworks para robots", como Player, YARP, Orocos, CARMEN, Orca, MOOS y Microsoft Robotics Studio.

El "grafo" de ejecución de ROS es una red de procesos punto a punto (potencialmente distribuidos entre varias máquinas) que están débilmente acoplados mediante la infraestructura de comunicación de ROS. ROS implementa varios estilos de comunicación, incluidos la comunicación síncrona tipo RPC a través de servicios, la transmisión asíncrona de datos mediante tópicos y el almacenamiento de datos en un Servidor de Parámetros. Estos conceptos se explican con mayor detalle en nuestra Visión General Conceptual.
En la actualidad existen dos versiones de ROS. Una  es conoccida como ROS1 o simplemente ROS y la segunda version es conocida como ROS2. 

## Sobre ROS1

ROS 1 (Robot Operating System) facilita la creación de sistemas complejos. Su arquitectura se basa en un grafo de ejecución donde los nodos (procesos independientes) se comunican mediante tópicos (publicación-suscripción) o servicios (solicitud-respuesta). El ROS Master coordina estas interacciones, gestionando nombres, registros y conexiones entre nodos. Además, incluye un servidor de parámetros para almacenar configuraciones globales. Los paquetes organizan los recursos del sistema, y las herramientas como roscore, rosrun y roslaunch simplifican la ejecución. Aunque ROS 1 es potente y flexible, depende del Master y no está optimizado para aplicaciones en tiempo real, lo que ha llevado a su evolución hacia ROS2.

La estructura de archivos y el grafo de ejecución de ROS son fundamentales para comprender cómo funciona este sistema en proyectos de robótica.

### Estructura de Archivos y Grafo de Ejecución en ROS

#### Estructura de Archivos en ROS

La estructura de archivos en ROS está diseñada para organizar los recursos de un proyecto robótico de manera eficiente. Los elementos principales incluyen:

- **Workspace (Espacio de trabajo):**
  El espacio de trabajo es la raíz donde se gestionan los paquetes de ROS. Por ejemplo, un típico workspace se organiza con las carpetas `src/`, `build/`, y `devel/`.
  
  - `src/`: Contiene el código fuente, generalmente organizado en paquetes.
  - `build/`: Carpeta generada automáticamente durante el proceso de compilación. Contiene archivos binarios intermedios.
  - `devel/`: Incluye configuraciones, bibliotecas y binarios listos para ejecutar, específicos del espacio de trabajo.
  
- **Paquetes (Packages):**
  Los paquetes son las unidades básicas de organización en ROS. Cada paquete incluye nodos, bibliotecas, archivos de configuración y otros recursos necesarios para una funcionalidad específica. Un paquete típico contiene:
  
  - `CMakeLists.txt`: Archivo de configuración para el sistema de compilación.
  - `package.xml`: Archivo de metadatos que define dependencias, autor, versión, etc.
  - Directorios como `scripts/`, `launch/`, `msg/`, `srv/` o `action/` para organizar scripts ejecutables, configuraciones de inicio, mensajes personalizados, servicios y acciones.

#### Grafo de Ejecución en ROS

El grafo de ejecución de ROS es una arquitectura distribuida y modular que permite la interacción entre los componentes de un sistema robótico. Sus principales características son:

- **Nodos (Nodes):**
  Los nodos son procesos que ejecutan tareas específicas en ROS. Estos pueden estar distribuidos en varias máquinas y comunicarse entre sí de forma transparente.

- **Tópicos (Topics):**
  Los nodos se comunican mediante la publicación y suscripción a tópicos. Esto permite el intercambio de mensajes de forma asincrónica y eficiente, ideal para flujos de datos continuos como sensores o cámaras.

- **Servicios (Services):**
  Ofrecen comunicación síncrona en un esquema de solicitud-respuesta, útil para operaciones que requieren confirmación inmediata, como iniciar o detener un proceso.

- **Servidor de Parámetros (Parameter Server):**
  Es una base de datos centralizada utilizada para almacenar configuraciones globales o parámetros accesibles por múltiples nodos.

- **Librerías y Herramientas:**
  ROS proporciona herramientas como `roscore` (que inicializa el sistema), `rosrun` (para ejecutar nodos individuales), y `roslaunch` (para iniciar múltiples nodos con configuraciones predefinidas).

El grafo de ejecución permite que los componentes estén débilmente acoplados, lo que fomenta la modularidad y escalabilidad en los sistemas robóticos. La implementación de esta arquitectura facilita el desarrollo colaborativo y la integración de nuevas funcionalidades sin necesidad de reestructurar todo el sistema.


# ¿Qué es ROS2?

ROS 2 (Robot Operating System 2) es la evolución de ROS, diseñado para superar las limitaciones del sistema original. Es un framework de código abierto que facilita el desarrollo de software modular, escalable y distribuido para robots. A diferencia de ROS 1, ROS 2 se centra en la seguridad, el rendimiento en tiempo real y la interoperabilidad, lo que lo hace más adecuado para aplicaciones industriales, colaborativas y críticas.

## **Características Clave de ROS 2**

### 1. **Arquitectura basada en DDS**
ROS 2 utiliza DDS (Data Distribution Service), un estándar para la comunicación de datos en sistemas distribuidos. Esto permite:
- Comunicación directa entre nodos sin depender de un nodo central.
- Mayor flexibilidad y robustez en redes distribuidas.
- Compatibilidad con entornos heterogéneos y multi-plataforma.

### 2. **Soporte en Tiempo Real**
ROS 2 está diseñado para trabajar con sistemas en tiempo real, lo que es esencial en aplicaciones críticas como la robótica médica o industrial.

### 3. **Seguridad**
Introduce mecanismos de seguridad como cifrado, autenticación y control de acceso, lo que permite aplicaciones en entornos sensibles.

### 4. **Multi-plataforma**
Es compatible con varios sistemas operativos como Linux, Windows y macOS, facilitando su adopción en diversos entornos de desarrollo.

### 5. **Modularidad y Escalabilidad**
ROS 2 permite una arquitectura más flexible, donde los nodos se pueden distribuir y escalar fácilmente en sistemas complejos.

## **Diferencias Principales con ROS 1**
| Característica           | ROS 1                             | ROS 2                           |
|--------------------------|------------------------------------|----------------------------------|
| Comunicación             | Basada en Master central          | Basada en DDS                   |
| Tiempo real              | Limitado                          | Soporte integrado               |
| Seguridad                | Básica                            | Cifrado y autenticación         |
| Multi-plataforma         | Principalmente Linux              | Linux, Windows, macOS           |
| Compatibilidad           | No retrocompatible con ROS 2      | Diseño modular y avanzado       |

## **Casos de Uso**
- Robótica industrial: Manipulación precisa y control en tiempo real.
- Robots autónomos: Vehículos autónomos y drones.
- Aplicaciones críticas: Robótica médica y sistemas colaborativos.

## **Ventajas**
- Mayor robustez y flexibilidad en sistemas distribuidos.
- Diseño moderno para satisfacer necesidades actuales de robótica.
- Enfoque en estándares y mejores prácticas industriales.

## **Conclusión**
ROS 2 representa un gran avance en el desarrollo de software robótico, abordando las limitaciones de ROS 1 y ofreciendo una plataforma más adecuada para entornos industriales y aplicaciones críticas. Su diseño modular y basado en estándares garantiza una larga vida útil y un ecosistema en constante expansión.



# 1- Install python
Python y el entorno Conda son herramientas ideales para trabajar con ROS 2 debido a su flexibilidad y facilidad de uso en entornos de desarrollo complejos. Python, como lenguaje interpretado, ofrece una sintaxis clara y una vasta cantidad de bibliotecas que facilitan el desarrollo de algoritmos para robótica. 


Conda permite gestionar entornos virtuales de forma eficiente, lo que resulta crucial para ROS 2, ya que sus dependencias pueden variar entre proyectos y sistemas operativos. Al combinar ambos, se logra un entorno controlado que minimiza conflictos de versiones y facilita la instalación de paquetes específicos necesarios para el desarrollo y la simulación en robótica. Esto no solo optimiza el flujo de trabajo, sino que también garantiza una mayor estabilidad y reproducibilidad en los proyectos.


### 2- Install ROS2
Install ROS2 humble and have it running. Here are some options depending in the OS. 
- **Ubuntu Jammy 22.04 LTS** from debian ource. [ros2 docs install tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **MacOS**: Use mamba and _robostack_ for installing the binaries for ros-humble. [robostack tutorial](https://robostack.github.io/GettingStarted.html)
- **Windows**: Use WSL for using a Linux Kernel and install ros as in Ubuntu OS . [here](https://learn.microsoft.com/es-es/windows/wsl/install)

> Rember to source the ros workspace

### Workspace config
Clone this repo and configure the workspace.
```bash
mkdir phantom_ws && cd phantom_ws
mkdir src && cd src
git clone https://github.com/labsir-un/phantomx-driver.git
cd ..
```
build and source the project 
```bash
colcon build --symlink-install && source install/setup.bash
```

Con esto terminamos la primera session!

## Additional tool recomended
- **VScode**
- **Iterm** Other terminal aplicattion
- **Arc** Browser for mac.
