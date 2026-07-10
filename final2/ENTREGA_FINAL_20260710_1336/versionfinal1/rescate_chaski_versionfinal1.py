#!/usr/bin/env python3
import math, time, csv, os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

def clip(x,a,b): return max(a,min(b,x))

class Rescate(Node):
    def __init__(self):
        super().__init__('rescate_chaski')
        for name,val in [
            ('mode','corridor'),('hand','left'),('v',0.035),('v_slow',0.022),('w',0.14),
            ('kp',0.12),('max_wz',0.08),('wall_target',0.30),('front_stop',0.46),
            ('front_danger',0.25),('side_danger',0.12),('turn_time',0.72),
            ('back_time',0.30),('start_straight_s',3.0),('pare_stop_s',3.0),
            ('force_first_left',True),('first_left_release',0.64),
            ('route','S4.0,L0.72,S2.5,R0.72,S2.5,L0.72,S3.0,R0.72,S3.0')]:
            self.declare_parameter(name,val)
        self.scan=None; self.pare=False
        self.front=self.left=self.right=self.fl=self.fr=float('inf')
        self.state='INIT'; self.until=0.0; self.t0=time.time(); self.act_t=time.time()
        self.turn_dir=1.0; self.route_i=0; self.route=self.parse_route(self.get_parameter('route').value)
        self.pares=0; self.pares_ok=0; self.last_pare=0.0; self.dead=0
        # FINAL2: la primera decision de giro del modo corridor se mantiene a la izquierda
        # hasta que el frente quede libre. Asi no cambia a derecha a mitad del giro.
        self.first_left_active=False; self.first_left_done=False
        self.last_odom=None; self.long_m=0.0
        self.pub_cmd=self.create_publisher(Twist,'/cmd_vel',10)
        self.pub_estado=self.create_publisher(String,'/maze/estado',10)
        self.pub_met=self.create_publisher(String,'/maze/metricas',10)
        self.create_subscription(LaserScan,'/scan',self.scan_cb,10)
        self.create_subscription(Bool,'/pare_detectado',self.pare_cb,10)
        self.create_subscription(Odometry,'/odom_raw',self.odom_cb,10)
        self.create_timer(0.10,self.loop)
        self.get_logger().info('FINAL2 listo: primera interseccion forzada a IZQUIERDA; PARE tiene prioridad.')

    def parse_route(self,s):
        out=[]
        for tok in s.split(','):
            tok=tok.strip().upper()
            if not tok: continue
            act=tok[0]
            try: dur=float(tok[1:])
            except: dur=0.0
            if act in 'SLRBP': out.append((act,dur))
        return out or [('S',3.0)]

    def pare_cb(self,msg): self.pare=bool(msg.data)

    def odom_cb(self,msg):
        x=msg.pose.pose.position.x; y=msg.pose.pose.position.y
        if self.last_odom:
            lx,ly=self.last_odom; d=math.hypot(x-lx,y-ly)
            if d<0.30: self.long_m += d
        self.last_odom=(x,y)

    def sector(self,msg,deg,width):
        c=math.radians(deg); w=math.radians(width)
        i0=int((c-w/2-msg.angle_min)/msg.angle_increment)
        i1=int((c+w/2-msg.angle_min)/msg.angle_increment)
        n=len(msg.ranges); i0=max(0,min(n-1,i0)); i1=max(0,min(n-1,i1))
        if i1<i0: i0,i1=i1,i0
        vals=[v for v in msg.ranges[i0:i1+1] if math.isfinite(v) and msg.range_min<=v<=msg.range_max]
        if not vals: return float('inf')
        vals.sort(); k=max(1,len(vals)//5)
        return sum(vals[:k])/k

    def scan_cb(self,msg):
        self.scan=msg
        self.front=self.sector(msg,0,24); self.fl=self.sector(msg,28,22); self.fr=self.sector(msg,-28,22)
        self.left=self.sector(msg,90,36); self.right=self.sector(msg,-90,36)

    def pub(self,v,w):
        t=Twist(); t.linear.x=float(v); t.angular.z=float(w); self.pub_cmd.publish(t)
    def stop(self): self.pub(0,0)

    def set_state(self,s,dur=0):
        if s!=self.state:
            self.get_logger().info(f'{self.state}->{s} F={self.front:.2f} L={self.left:.2f} R={self.right:.2f} PARE={self.pare}')
        self.state=s; self.until=time.time()+dur; self.act_t=time.time()

    def choose_turn(self):
        # si un lado está claramente libre, usarlo; si no, mano elegida
        if abs(self.left-self.right)>0.20:
            return 1.0 if self.left>self.right else -1.0
        return 1.0 if self.get_parameter('hand').value=='left' else -1.0

    def center_wz(self,kp,max_wz,target,side_danger):
        l_ok=math.isfinite(self.left) and self.left<1.2
        r_ok=math.isfinite(self.right) and self.right<1.2
        wz=0.0
        if l_ok and r_ok:
            wz=kp*(self.left-self.right)       # pasillo: centrar
        elif l_ok:
            wz=kp*(self.left-target)          # seguir pared izquierda
        elif r_ok:
            wz=kp*(target-self.right)         # seguir pared derecha
        if self.left<side_danger: wz=-abs(max_wz)
        if self.right<side_danger: wz=abs(max_wz)
        return clip(wz,-max_wz,max_wz)

    def loop(self):
        if self.scan is None:
            self.stop(); return
        now=time.time()
        mode=self.get_parameter('mode').value
        v=float(self.get_parameter('v').value); vs=float(self.get_parameter('v_slow').value)
        w=float(self.get_parameter('w').value); kp=float(self.get_parameter('kp').value)
        mw=float(self.get_parameter('max_wz').value); target=float(self.get_parameter('wall_target').value)
        fs=float(self.get_parameter('front_stop').value); fd=float(self.get_parameter('front_danger').value)
        sd=float(self.get_parameter('side_danger').value); tt=float(self.get_parameter('turn_time').value)
        bt=float(self.get_parameter('back_time').value); pstop=float(self.get_parameter('pare_stop_s').value)
        start=float(self.get_parameter('start_straight_s').value)

        if self.pare and now-self.last_pare>6 and self.state!='PARE':
            self.pares+=1; self.pares_ok+=1; self.last_pare=now; self.set_state('PARE',pstop)
        if self.state=='PARE':
            self.stop()
            if now>=self.until: self.set_state('CORRIDOR' if mode=='corridor' else 'ROUTE')
            self.status(); return

        # emergencia: solo frente/frente diagonal muy cerca; paredes laterales NO son obstaculo
        if self.state not in ('BACK','TURN') and (self.front<fd or self.fl<0.16 or self.fr<0.16):
            self.dead+=1
            force_first_left=bool(self.get_parameter('force_first_left').value)
            if mode=='corridor' and force_first_left and not self.first_left_done:
                if not self.first_left_active:
                    self.get_logger().warn('FINAL2: PRIMERA INTERSECCION/DECISION FORZADA A IZQUIERDA')
                self.first_left_active=True
                self.turn_dir=1.0
            else:
                self.turn_dir=self.choose_turn()
            self.set_state('BACK',bt); self.status(); return
        if self.state=='BACK':
            self.pub(-0.025,0)
            if now>=self.until: self.set_state('TURN',tt)
            self.status(); return
        if self.state=='TURN':
            self.pub(0,self.turn_dir*w)
            if now>=self.until: self.set_state('CORRIDOR' if mode=='corridor' else 'ROUTE')
            self.status(); return

        if now-self.t0 < start and self.front>fs:
            self.set_state('START_RECTO')
            self.pub(vs, self.center_wz(kp*0.5,mw*0.6,target,sd))
            self.status(); return

        if mode=='route':
            self.route_loop(v,vs,w,fs)
        else:
            self.set_state('CORRIDOR')
            force_first_left=bool(self.get_parameter('force_first_left').value)
            first_left_release=float(self.get_parameter('first_left_release').value)

            # FINAL2: en la primera decision no basta con ordenar un solo pulso a la izquierda.
            # Se conserva esa direccion durante todos los pulsos de giro hasta que el frente
            # quede libre; luego el algoritmo vuelve a su comportamiento original.
            if self.first_left_active:
                if self.front>=first_left_release:
                    self.first_left_active=False
                    self.first_left_done=True
                    self.get_logger().warn('FINAL2: PRIMER GIRO A IZQUIERDA COMPLETADO')
                    vx=vs if self.front<fs+0.18 else v
                    self.pub(vx,self.center_wz(kp,mw,target,sd))
                else:
                    self.turn_dir=1.0
                    self.set_state('TURN',tt)
            elif self.front<fs:
                if force_first_left and not self.first_left_done:
                    self.first_left_active=True
                    self.turn_dir=1.0
                    self.get_logger().warn('FINAL2: PRIMERA INTERSECCION/DECISION FORZADA A IZQUIERDA')
                else:
                    self.turn_dir=self.choose_turn()
                self.set_state('TURN',tt)
            else:
                vx=vs if self.front<fs+0.18 else v
                self.pub(vx,self.center_wz(kp,mw,target,sd))
        self.status()

    def route_loop(self,v,vs,w,fs):
        now=time.time()
        if self.route_i>=len(self.route):
            self.set_state('FIN'); self.stop(); return
        act,dur=self.route[self.route_i]
        if now-self.act_t>=dur:
            self.route_i+=1; self.act_t=now; self.stop(); return
        if act=='S' and self.front<fs and now-self.act_t>0.6:
            self.route_i+=1; self.act_t=now; self.stop(); return
        if act=='S': self.pub(v,self.center_wz(0.07,0.05,0.30,0.12))
        elif act=='L': self.pub(0,abs(w))
        elif act=='R': self.pub(0,-abs(w))
        elif act=='B': self.pub(-vs,0)
        else: self.stop()

    def status(self):
        s=String(); s.data=self.state; self.pub_estado.publish(s)
        m=String()
        m.data=f'estado={self.state}, t={time.time()-self.t0:.1f}, F={self.front:.2f}, L={self.left:.2f}, R={self.right:.2f}, ruta_cm={self.long_m*100:.1f}, PARE={self.pare}, pare_detectados={self.pares}, pare_respetados={self.pares_ok}, dead_ends={self.dead}, first_left_active={self.first_left_active}, first_left_done={self.first_left_done}, route_i={self.route_i}'
        self.pub_met.publish(m)

    def save_csv(self):
        path='/root/yahboomcar_ws/metricas_granprix.csv'
        os.makedirs(os.path.dirname(path),exist_ok=True)
        new=not os.path.exists(path)
        with open(path,'a',newline='') as f:
            w=csv.writer(f)
            if new: w.writerow(['ronda','llego_meta','tiempo_s','long_ruta_cm','long_optima_cm','eficiencia','colisiones','pare_reales','pare_detectados','pare_respetados','pare_falsos','dead_ends_visitados','karpinchus_rodeados'])
            w.writerow([1,'No',round(time.time()-self.t0,2),round(self.long_m*100,1),'','','','',self.pares,self.pares_ok,0,self.dead,0])

def main():
    rclpy.init()
    n=Rescate()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally:
        try: n.stop(); n.save_csv(); time.sleep(0.1)
        except Exception as e: print(e)
        n.destroy_node()
        try: rclpy.shutdown()
        except Exception: pass

if __name__=='__main__': main()
