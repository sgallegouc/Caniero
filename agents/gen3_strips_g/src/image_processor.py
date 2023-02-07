import cv2
import numpy as np
import apriltag
from scipy.spatial.transform import Rotation as R

class ImageProcessor():
    def __init__(self):
        self.detector = apriltag.Detector(apriltag.DetectorOptions(families="tag16h5"))

    def extract_color_and_depth(self, color_raw, depth_raw):
        depth = np.frombuffer(depth_raw, dtype=np.uint16)
        depth = depth.reshape((480, 640))

        color = np.frombuffer(color_raw, dtype=np.uint8)
        color = color.reshape((480, 640, 3))
        color = cv2.resize(color, (1280, 960), interpolation=cv2.INTER_NEAREST)
        
        return color, depth

    def compute_april_tags(self, color, depth, focals):
        tags = self.detector.detect(cv2.cvtColor(color, cv2.COLOR_RGB2GRAY))
        if len(tags) == 0:
            print("Compute_april_tags: No tags detected")
        # else:
        #     for tag in tags:
        #         for idx in range(len(tag.corners)):
        #             cv2.line(img, tuple(tag.corners[idx - 1, :].astype(int)), tuple(tag.corners[idx, :].astype(int)), (0, 255, 0))
        #             cv2.putText(img, str(tag.tag_id),
        #                        org = (tag.corners[0, 0].astype(int) + 10, tag.corners[0, 1].astype(int) + 10),
        #                        fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8, color=(0, 0, 255))
        #             cv2.rectangle(img, tuple(tag.center.astype(int) - 1), tuple(tag.center.astype(int) + 1), (255, 0, 255))
        simple_tags = self.__simplify_tags(tags, color, depth, focals)
        return tags, simple_tags

    def __simplify_tags(self, tags, color, depth, focals):
        s_tags = {}
        for tag in tags:
            mat = self.detector.detection_pose(tag,[focals[0], focals[1], 640, 480], 0.04)
            rot = mat[0][:3,:3]
            r = R.from_matrix(rot)
            rot = np.multiply(r.as_euler('yxz'), -1).tolist() # [r_x, r_y, r_z]
            rot[-1] *= -1

            index_x = int(tag.center[1]) // 2
            index_y = int(tag.center[0]) // 2
            pos_z = np.mean(depth[index_x-10:index_x+10,index_y-10:index_y+10])
            pos_x = - ((tag.center[1]//2 - color.shape[0] // 4) * pos_z) / focals[0]  
            pos_y = - ((tag.center[0]//2 - color.shape[1] // 4) * pos_z) / focals[1]
            pos = [pos_y, pos_x, pos_z] # Acording to gripper reference frame
            # pos = [pos_x, pos_y, pos_z] # Acording to world reference frame
            
            s_tags[tag.tag_id] = {"pos": pos, "rot": rot}
        return s_tags