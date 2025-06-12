from modulo_components.component import Component
import state_representation as sr
import serial
import time
import ezdxf
import math
import json
from threading import Event

# ======== CONFIGURATION ========
SCALE_FACTOR = 1.0       # Facteur de mise à l'échelle
OFFSET_X = 0          # Décalage sur l'axe X
OFFSET_Y = 0       # Décalage sur l'axe Y
LASER_POWER = 500       # Puissance du laser (0 à 1000)
FEED_RATE = 600          # Vitesse de déplacement en mm/min


class CrealityInterface(Component):
    def __init__(self, node_name: str, *args, **kwargs):
        super().__init__(node_name, *args, **kwargs)
        
        self.add_parameter(sr.Parameter("usb_path", sr.ParameterType.STRING), "USB path")
        self.add_parameter(sr.Parameter("baud_rate", 115200, sr.ParameterType.INT), "Baud rate")

        self._serial = serial.Serial(self.get_parameter_value("usb_path"), self.get_parameter_value("baud_rate"))
        self.send_wake_up()

        self.add_predicate("laser_finished", False)
        self.add_service("run_gcode", self._run_gcode)

    def _run_gcode(self, msg):
        self.set_predicate("laser_finished", False)
        # Affiche la demande reçue
        self.get_logger().info(f"Received service call with payload {msg}")

        """if isinstance(msg, str):
            # Si msg est une chaîne de caractères, essaie de la convertir en dictionnaire (si c'est du JSON par exemple)
            try:
                msg = json.loads(msg)  # Supposons que msg est un JSON
            except json.JSONDecodeError:
                self.get_logger().error("Le message n'est pas un format JSON valide.")
                return {"success": False, "message": "Le message n'est pas un format JSON valide."}
        
        if isinstance(msg, dict):
            # Vérifier que le chemin du fichier DXF est bien dans le message
            dxf_file_path = msg.get("dxf_path")
            if not dxf_file_path:
                self.get_logger().error("Le fichier DXF n'a pas été fourni.")
                return {"success": False, "message": "Le fichier DXF n'a pas été fourni."}
        else:
            self.get_logger().error("Le message reçu n'est ni un dictionnaire ni une chaîne JSON valide.")
            return {"success": False, "message": "Le message reçu est dans un format invalide."}
        """
        # Convertir le fichier DXF en G-code
        gcode_lines = self._dxf_to_gcode(msg)
        if not gcode_lines:
            self.get_logger().error(f"Erreur lors de la conversion du fichier {msg}.")
            return {"success": False, "message": "Erreur lors de la conversion du fichier DXF."}

        # Envoyer le G-code à la machine
        self.send_gcode_to_machine(gcode_lines)
        time.sleep(2)
        self.set_predicate("laser_finished", True)
        return {"success": True, "message": "G-code envoyé avec succès."}


    def send_gcode_to_machine(self, gcode_lines):
        try:
            with serial.Serial(self.get_parameter_value("usb_path"), self.get_parameter_value("baud_rate"), timeout=1) as ser:
                time.sleep(2)
                ser.write(b"\r\n\r\n")  # "Réveiller" le contrôleur
                time.sleep(2)
                ser.flushInput()

                self.initialize_machine(ser, do_homing=True)  # Active le homing automatiquement

                for line in gcode_lines:
                    command = line.strip() + '\n'
                    ser.write(command.encode('utf-8'))
                    while True:
                        response = ser.readline()
                        if response:
                            decoded = response.strip().decode()
                            self.get_logger().debug(f"Réponse: {decoded}")
                            if decoded == "ok" or "error" in decoded:
                                break
                        time.sleep(0.01)  # Petite pause pour laisser GRBL traiter

                # Attendre que la machine soit vraiment Idle
                self.wait_until_idle(ser)
                self.set_predicate("laser_finished", True)

        except serial.SerialException as e:
            self.get_logger().error(f"Erreur de communication série: {e}")

    def wait_until_idle(self, ser, timeout=120):
        """Attend que la machine GRBL soit en état Idle (fin du job)."""
        self.get_logger().warning("Attente que la machine soit Idle...")
        start_time = time.time()
        idle_counter = 0
        while True:
            ser.write(b'?\n')
            grbl_out = ser.readline()
            grbl_response = grbl_out.strip().decode('utf-8')
            self.get_logger().warning(f"Réponse GRBL: {grbl_response}")
            if "ok" in grbl_response or "Idle" in grbl_response:
                idle_counter += 1
            else:
                idle_counter = 0
            if idle_counter > 20:
                break
            if time.time() - start_time > timeout:
                self.get_logger().warning("Timeout en attendant que la machine soit Idle.")
                break
            time.sleep(0.2)

    def on_validate_parameter_callback(self, parameter: sr.Parameter) -> bool:
        if parameter.get_name() == "usb_path" and parameter.is_empty():
            self.get_logger().error("Provide a non empty value for parameter 'usb_path'")
            return False
        return True
    
    #q6uCINCoaVKV1c9IkKwx

    #e4396706-b9a5-4f62-ad89-6198ee98d6fc.Xw19Ca0q1aD1CxHVtrSdMvna2yMHco6502t9Qnk8CUxj

    def send_wake_up(self):
        # Wake up, hit enter a few times to wake the Printrbot
        self._serial.write(str.encode("\r\n\r\n"))
        time.sleep(1)   # Wait for Printrbot to initialize
        self._serial.flushInput()  # Flush startup text in serial input

    def remove_comment(string):
        if (string.find(';') == -1):
            return string
        else:
            return string[:string.index(';')]

    def remove_eol_chars(string):
        # removed \n or traling spaces
        return string.strip()
    
    def wait_for_movement_completion(self, cleaned_line):
        Event().wait(1)
        if cleaned_line != '$X' or '$$':
            idle_counter = 0
            while True:
                # Event().wait(0.01)
                self._serial.reset_input_buffer()
                command = str.encode('?' + '\n')
                self._serial.write(command)
                grbl_out = self._serial.readline() 
                grbl_response = grbl_out.strip().decode('utf-8')

                if grbl_response != 'ok':

                    if grbl_response.find('Idle') > 0:
                        idle_counter += 1

                if idle_counter > 10:
                    break
    
    def _apply_offset(self, x, y):
        x_offset = x * SCALE_FACTOR + OFFSET_X
        y_offset = y * SCALE_FACTOR + OFFSET_Y
        self.get_logger().info(f"Original: ({x}, {y}), Offset: ({x_offset}, {y_offset})")
        return x_offset, y_offset

    def _laser_on(self, power=LASER_POWER):
        return f"M3 S{power}"

    def _laser_off(self):
        return "M5"

    def _rapid_move(self, x, y):
        return f"G0 X{x:.3f} Y{y:.3f}"

    def _engrave_move(self, x, y):
        return f"G1 X{x:.3f} Y{y:.3f} F{FEED_RATE}"
    
    def _approximate_arc(self, x0, y0, x1, y1, bulge, segments=10):
        arc_pts = []
        for i in range(1, segments):
            t = i / segments
            xt = (1 - t) * x0 + t * x1
            yt = (1 - t) * y0 + t * y1
            arc_pts.append((xt, yt))
        arc_pts.append((x1, y1))
        return arc_pts


    def _dxf_to_gcode(self, filename):
        try:
            doc = ezdxf.readfile(filename)
        except IOError:
            self.get_logger().error(f"Erreur lors de l'ouverture du fichier {filename}.")
            return []
        except ezdxf.DXFStructureError:
            self.get_logger().error(f"Le fichier {filename} n'est pas un fichier DXF valide.")
            return []

        msp = doc.modelspace()
        gcode = ["G21", "G90", self._laser_off()]  # Init de base

        # Traitement des entités DXF
        for entity in msp:
            self.get_logger().info(f"Entity Type: {entity.dxftype()}, Data: {entity.dxf}")

            if entity.dxftype() == "LINE":
                self.get_logger().info(f"Entity: {entity}")
                x1, y1 = self._apply_offset(entity.dxf.start.x, entity.dxf.start.y)
                x2, y2 = self._apply_offset(entity.dxf.end.x, entity.dxf.end.y)
                #self.get_logger().info(f"LINE Start: ({entity.dxf.start.x}, {entity.dxf.start.y}), "
                                       #f"End: ({entity.dxf.end.x}, {entity.dxf.end.y}), "
                                       #f"Transformed Start: ({x1}, {y1}), End: ({x2}, {y2})")
                gcode.append(self._laser_off())
                gcode.append(self._rapid_move(x1, y1))
                #self.get_logger().info(f"G-code: {gcode[-1]}")
                gcode.append(self._laser_on())
                gcode.append(self._engrave_move(x2, y2))
                #self.get_logger().info(f"G-code: {gcode[-1]}")
                gcode.append(self._laser_off())

            elif entity.dxftype() == "ARC":
                self.get_logger().info(f"Entity: {entity}")
                cx, cy = self._apply_offset(entity.dxf.center.x, entity.dxf.center.y)
                radius = entity.dxf.radius * SCALE_FACTOR
                start_angle = math.radians(entity.dxf.start_angle)
                end_angle = math.radians(entity.dxf.end_angle)

                #self.get_logger().info(f"ARC Center: ({entity.dxf.center.x}, {entity.dxf.center.y}), "
                                       #f"Radius: {entity.dxf.radius}, "
                                       #f"Start Angle: {entity.dxf.start_angle}, End Angle: {entity.dxf.end_angle}")

                num_points = 200
                points = []
                for i in range(num_points + 1):
                    angle = start_angle + (end_angle - start_angle) * (i / num_points)
                    x = cx + radius * math.cos(angle)
                    y = cy + radius * math.sin(angle)
                    points.append((x, y))
                    #self.get_logger().info(f"Point {i}: ({x}, {y})")

                if points:
                    gcode.append(self._laser_off())
                    gcode.append(self._rapid_move(points[0][0], points[0][1]))
                    gcode.append(self._laser_on())
                    for x, y in points[1:]:
                        gcode.append(self._engrave_move(x, y))
                    gcode.append(self._laser_off())

            elif entity.dxftype() == "CIRCLE":
                self.get_logger().info(f"Entity: {entity}")
                cx, cy = self._apply_offset(entity.dxf.center.x, entity.dxf.center.y)
                radius = entity.dxf.radius * SCALE_FACTOR

                num_points = 200
                points = [(cx + radius * math.cos(2 * math.pi * i / num_points),
                           cy + radius * math.sin(2 * math.pi * i / num_points)) for i in range(num_points + 1)]

                if points:
                    gcode.append(self._laser_off())
                    gcode.append(self._rapid_move(points[0][0], points[0][1]))
                    gcode.append(self._laser_on())
                    for x, y in points[1:]:
                        gcode.append(self._engrave_move(x, y))
                    gcode.append(self._laser_off())

            elif entity.dxftype() == "ELLIPSE":
                self.get_logger().info(f"Entity: {entity}")

                center = entity.dxf.center
                major_axis = entity.dxf.major_axis
                ratio = entity.dxf.ratio 
                start_param = entity.dxf.start_param
                end_param = entity.dxf.end_param

                num_points = 200
                points = []

                # Longueur du vecteur major_axis
                major_length = math.hypot(major_axis.x, major_axis.y)

                for i in range(num_points + 1):
                    t = start_param + (end_param - start_param) * i / num_points
                    cos_t = math.cos(t)
                    sin_t = math.sin(t)

                    # Point sur ellipse en coordonnées locales
                    x_ell = major_length * cos_t
                    y_ell = major_length * ratio * sin_t

                    # Rotation selon direction du vecteur major_axis
                    angle = math.atan2(major_axis.y, major_axis.x)
                    x_rot = x_ell * math.cos(angle) - y_ell * math.sin(angle)
                    y_rot = x_ell * math.sin(angle) + y_ell * math.cos(angle)

                    # Position globale avec offset
                    x = center.x + x_rot
                    y = center.y + y_rot
                    x_off, y_off = self._apply_offset(x, y)
                    points.append((x_off, y_off))
                    #self.get_logger().info(f"Ellipse point {i}: ({x_off}, {y_off})")

                if points:
                    gcode.append(self._laser_off())
                    gcode.append(self._rapid_move(points[0][0], points[0][1]))
                    gcode.append(self._laser_on())
                    for x, y in points[1:]:
                        gcode.append(self._engrave_move(x, y))
                    gcode.append(self._laser_off())


            elif entity.dxftype() == "LWPOLYLINE":
                self.get_logger().info(f"Entity: {entity}")
              
                vertices = list(entity.vertices())  # [(x, y, start_width, end_width, bulge), ...]
                if vertices:
                    x0, y0 = self._apply_offset(vertices[0][0], vertices[0][1])
                    gcode.append(self._laser_off())
                    self.get_logger().info(f"Déplacement vers le point de départ: ({x0}, {y0})")
                    gcode.append(self._rapid_move(x0, y0))
                    gcode.append(self._laser_on())


                    for i in range(1, len(vertices)):
                        x1_raw, y1_raw = vertices[i][0], vertices[i][1]
                        x1, y1 = self._apply_offset(x1_raw, y1_raw)
                        bulge = vertices[i - 1][4] if len(vertices[i - 1]) >= 5 else 0.0

                        if bulge != 0.0:
                            arc_points = self._approximate_arc(vertices[i - 1][0], vertices[i - 1][1],
                                                            x1_raw, y1_raw, bulge)
                            for px, py in arc_points:
                                px_off, py_off = self._apply_offset(px, py)
                                gcode.append(self._engrave_move(px_off, py_off))
                        else:
                            gcode.append(self._engrave_move(x1, y1))

                    # Fermeture si nécessaire
                    if entity.closed:
                        gcode.append(self._engrave_move(x0, y0))

                    gcode.append(self._laser_off())


            elif entity.dxftype() == "POINT":
                self.get_logger().info(f"Entity: {entity}")
                x, y = self._apply_offset(entity.dxf.location.x, entity.dxf.location.y)
                gcode.append(self._laser_off())
                gcode.append(self._rapid_move(x, y))
                gcode.append(self._laser_on())


        gcode.append(self._laser_off())
        gcode.append("G0 X0 Y0")  # Retour origine
        return gcode

    def initialize_machine(self, ser, do_homing=True):
        print("Initialisation de la machine...")
        if do_homing:
            ser.write(b"$H\n")  # Lancer le homing
            time.sleep(5)
        ser.write(b"$X\n")  # Déverrouiller la machine si nécessaire
        time.sleep(0.5)
        ser.write(b"G21\n")  # Unités en mm
        time.sleep(0.1)
        ser.write(b"G90\n")  # Mode absolu
        time.sleep(0.1)
        ser.write(b"M5\n")   # Laser OFF
        time.sleep(0.1)
        ser.write(b"G92 X0 Y0\n")  # Position de départ à (0,0)
        time.sleep(0.1)
        print("Machine initialisée et prête.")
