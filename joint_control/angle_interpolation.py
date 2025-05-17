'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import hello
from keyframes import leftBackToStand
from keyframes import leftBellyToStand
from keyframes import rightBackToStand
from keyframes import rightBellyToStand
from keyframes import wipe_forehead
import logging

#logging.basicConfig(filename="agent_log.txt", level=logging.DEBUG) #for debugging

class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])
        self.start_time = None # initializing the time when we startet our agent
        self.prev_keyframe = None   # for resetting the start_time

    def think(self, perception):
        #logging.debug(perception.time)         # tried to see what simulation time we have and saw its way over the keyframe times

        if self.keyframes != self.prev_keyframe:
            self.start_time = float(perception.time) # save the time we started new keyframe motion (Probblem when the robot doesnt stand up and fall on back or belle again a second time so the starttime wont be resettet)
            self.prev_keyframe = self.keyframes

        #if self.start_time is None:
           # self.start_time = perception.time   # save the time we started our agent (very first time) before standing_up.py

        relative_time = float(perception.time) - self.start_time   # the simulation time - the time we started our agent is the time we have rn to let the robot move by their keyframes
        target_joints = self.angle_interpolation(self.keyframes, relative_time)

        # hello() doesnt have LHipYawPitch which is correct because it doesnt need that but the code wont compile if we tried to acces that so i removed that only for hello()
        #target_joints['RHipYawPitch'] = target_joints['LHipYawPitch'] # copy missing joint in keyframes 
        if 'LHipYawPitch' in target_joints:
            target_joints['RHipYawPitch'] = target_joints['LHipYawPitch']
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def bezier(self, p0, p1, p2, p3, t):
        """Cubic Bezier Interpolation"""
        return (
            (1 - t) ** 3 * p0 +
            3 * (1 - t) ** 2 * t * p1 +         # from the lecture
            3 * (1 - t) * t ** 2 * p2 +
            t ** 3 * p3
        )

    def get_angle_at_time(self, times, keys, t_now):
        t_now = float(t_now)
        for i in range(len(times) - 1):
            t0 = times[i]
            t1 = times[i + 1]

            if t0 <= t_now <= t1:
                k0 = keys[i]
                k1 = keys[i + 1]

                if isinstance(k0, float) or isinstance(k1, float):
                    # Kein Bezier, also lineare Interpolation oder direkt zurückgeben
                    a0 = k0 if isinstance(k0, float) else k0[0]
                    a1 = k1 if isinstance(k1, float) else k1[0]
                    dt = t1 - t0
                    t = (t_now - t0) / dt
                    return (1 - t) * a0 + t * a1  # einfache lineare Interpolation

                # Bezier-Interpolation
                a0, h0_1, h0_2 = k0
                a1, h1_1, h1_2 = k1

                dt = t1 - t0
                t = (t_now - t0) / dt

                h1_angle = a0 + h0_2[2]
                h2_angle = a1 + h1_1[2]

                return self.bezier(a0, h1_angle, h2_angle, a1, t)

        # Falls t_now > alle Zeiten, nimm letzten Wert
        last_key = keys[-1]
        return last_key if isinstance(last_key, float) else last_key[0]

    def angle_interpolation(self, keyframes, perception):
        # YOUR CODE HERE
        target_joints = {}
        names, times_list, keys_list = keyframes

        current_time = perception   # which is the relative time calulated above (i tried to use perception.time first but that would be the simulation time
                                    # which is always above the timestamps for the robot because i cant start the agent at the same time as the software)

        for name, times, keys in zip(names, times_list, keys_list):
            angle = self.get_angle_at_time(times, keys, current_time)
            target_joints[name] = angle

        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = rightBackToStand()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()



