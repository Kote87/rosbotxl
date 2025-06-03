#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy, math, numpy as np, cv2, random
from rclpy.node        import Node
from rclpy.action      import ActionClient
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg      import OccupancyGrid, Odometry
from nav2_msgs.action  import NavigateToPose, BackUp
from action_msgs.msg   import GoalStatus

STUCK_D, STUCK_T = 0.10, 8.0      # 10 cm / 8 s ⇒ Back-Up 0.2 m
BACK_DIST        = 0.20
FRONTIER_RES_M   = 0.25           # agrupa todo lo que esté <25 cm
UNKNOWN          = -1

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        self.nav  = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.bkup = ActionClient(self, BackUp,        'back_up')
        self.create_subscription(OccupancyGrid,'/global_costmap/costmap',
                                 self._map_cb,10)
        self.create_subscription(Odometry,'/odometry/filtered',
                                 self._odom_cb,20)
        self.info=None; self.grid=None; self.pose=None
        self.goal_act=False; self.backing=False
        self.last_mv=self.get_clock().now()
        self.create_timer(1.0,self._tick)

    # ---------- Callbacks ----------
    def _map_cb(self,msg):
        self.info=msg.info
        self.grid=np.frombuffer(msg.data,dtype=np.int8)\
                    .reshape(msg.info.height,msg.info.width)

    def _odom_cb(self,msg):
        p=msg.pose.pose.position; q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.pose and math.hypot(p.x-self.pose[0],p.y-self.pose[1])>STUCK_D:
            self.last_mv=self.get_clock().now()
        self.pose=(p.x,p.y,yaw)

    # ---------- Main loop ----------
    def _tick(self):
        if self.backing: return
        if ( self.goal_act and
             (self.get_clock().now()-self.last_mv).nanoseconds*1e-9>STUCK_T ):
            self._do_backup(); return
        if not self.goal_act: self._send_goal()

    # ---------- Frontier detection ----------
    def _compute_frontiers(self):
        if self.grid is None: return []
        free=np.where(self.grid<10,1,0).astype(np.uint8)
        # vecinos desconocidos
        unk_mask=(self.grid==UNKNOWN).astype(np.uint8)
        dil=cv2.dilate(unk_mask,np.ones((3,3),np.uint8))
        frontiers=(dil & free).astype(np.uint8)
        ys,xs=np.where(frontiers==1)
        pts=list(zip(ys,xs))
        return pts

    def _cluster_frontiers(self,pts):
        res=self.info.resolution
        step=int(FRONTIER_RES_M/res)
        if step<1: step=1
        clusters={}
        for y,x in pts:
            key=(y//step,x//step)
            clusters.setdefault(key,[]).append((y,x))
        # devuelve el centro del cluster
        ctrs=[]
        for cl in clusters.values():
            ys,xs=zip(*cl)
            ctrs.append( (int(np.mean(ys)),int(np.mean(xs)),len(cl)) )
        return ctrs

    # ---------- Choose & send goal ----------
    def _send_goal(self):
        if not (self.nav.server_is_ready() and self.info and self.pose): return
        fpts=self._compute_frontiers()
        if not fpts:
            self.get_logger().info('Sin frontiers → exploración acabada')
            return
        clusters=self._cluster_frontiers(fpts)
        x0,y0,_=self.pose
        # elige cluster más cercano
        def dist_sq(c):
            y,x,_=c
            gx=x*self.info.resolution+self.info.origin.position.x
            gy=y*self.info.resolution+self.info.origin.position.y
            return (gx-x0)**2+(gy-y0)**2
        cy,cx,_=min(clusters,key=dist_sq)
        goal=self._cell_to_pose(cy,cx)
        ng=NavigateToPose.Goal(); ng.pose=goal
        self.goal_act=True
        self.nav.send_goal_async(ng).add_done_callback(self._goal_resp)

    def _cell_to_pose(self,y,x):
        pose=PoseStamped()
        pose.header.frame_id='map'
        pose.header.stamp=self.get_clock().now().to_msg()
        pose.pose.position.x= x*self.info.resolution+self.info.origin.position.x \
                              + self.info.resolution/2
        pose.pose.position.y= y*self.info.resolution+self.info.origin.position.y \
                              + self.info.resolution/2
        pose.pose.orientation.w=1.0
        return pose

    def _goal_resp(self,fut):
        h=fut.result()
        if not h or not h.accepted:
            self.goal_act=False; return
        h.get_result_async().add_done_callback(self._goal_done)
    def _goal_done(self,fut):
        self.goal_act=False

    # ---------- Back-Up ----------
    def _do_backup(self):
        if not self.bkup.server_is_ready(): self.goal_act=False; return
        g=BackUp.Goal()
        g.target=Point(x=-BACK_DIST,y=0,z=0); g.speed=0.10; g.time_allowance.sec=4
        self.backing=True
        self.bkup.send_goal_async(g).add_done_callback(
            lambda _: (setattr(self,'backing',False),
                       setattr(self,'goal_act',False),
                       setattr(self,'last_mv',self.get_clock().now())) )

def main(args=None):
    rclpy.init(args=args); rclpy.spin(FrontierExplorer()); rclpy.shutdown()
if __name__=='__main__': main()
