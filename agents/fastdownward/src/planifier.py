from xml.dom.pulldom import END_DOCUMENT
import numpy as np
from numpy import linalg as LA
import os

class Planifier():
    def __init__(self):
        return

    def create_initial_state(self, tags):
        initState = ["  (handempty)"]
        cubes = []
        for tag in tags:
            # if tag.tag_id == 1:
            #     initState.append(f"  (clear {1})")
            #     initState.append(f"     (on {1} {2})")
            # elif tag.tag_id == 2:
            #     initState.append(f"     (ontable {2})")
            # else:
            initState.append(f"  (clear {tag.tag_id})")
            initState.append(f"     (ontable {tag.tag_id})")
                
            cubes.append(tag.tag_id)
        return initState, cubes

    def create_initial_state_cubes(self, cube_ids):
        initState = ["  (handempty)"]
        cubes = []
        for name in cube_ids:
            initState.append(f"  (clear {name})")
            initState.append(f"     (ontable {name})")
                
            cubes.append(name)
        return initState, cubes


    def create_current_state(self, states):
        initState = ["  (handempty)"]
        cubes = set([])

        for s in states:
            cond = s[0]

            if cond == "clear":
                initState.append(f"  (clear {s[1]})")
                cubes.add(s[1])

            elif cond == "table":
                initState.append(f"     (ontable {s[1]})")
                cubes.add(s[1])

            elif cond == "on":
                initState.append((f"     (on {s[1]} {s[2]})")) 
                cubes.add(s[2])
        
        return initState, list(cubes)

    def is_horizontal(self, tag):
        up_line = LA.norm(np.array(tag.corners[3]) - np.array(tag.corners[2]))
        down_line = LA.norm(np.array(tag.corners[1]) - np.array(tag.corners[0]))
        return down_line > up_line

    def create_final_state(self, goal):
        end_state = ["  (and(handempty)"]

        end_state.append(f"     (ontable {4})")
        # end_state.append(f"     (ontable {5})")
        # end_state.append(f"     (ontable {3})")
        end_state.append(f"     (ontable {6})")
        end_state.append((f"     (on {5} {3})"))  
        end_state.append((f"     (on {3} {4})"))       
     
        return end_state

    def save_to_file(self, init_state, end_state, tag_list):
        tag_list.sort()
        blocks = ""
        for tag in tag_list:
            blocks += str(tag) + " "

        # for r in init_state:
        #     print(r)
        # for r in end_state:
        #     print(r)
        lines = [
            "(define (problem BLOCKS-{}-1)".format(len(tag_list)),
            "(:domain blocks)",
            "(:objects {}- block)".format(blocks),
            "(:init",
            *init_state,
            ")",
            "(:goal",
            *end_state, 
            "   )",
            ")",
            ")"
        ]

        print("FILE", '\n'.join(lines))

        with open("init_state.pdll", 'w+') as f:
            f.write('\n'.join(lines))
    
    def exec_planner(self):
        #print("PLANNER EXECUTION")
        os.system("cd /home/robocomp/robocomp/software/fastDownward/fast-downward-20.06/ && ls && \
        ./fast-downward.py --alias lama-first /home/robocomp/robocomp/components/manipulation_kinova_gen3/etc/domain.pddl {}"
        .format("/home/robocomp/robocomp/components/manipulation_kinova_gen3/agents/fastdownward/init_state.pdll"))
    
    def load_plan(self):
        plan = []
        with open("/home/robocomp/robocomp/software/fastDownward/fast-downward-20.06/sas_plan", 'r+') as f:
            lines = f.readlines()
            for line in lines:
                if ";" not in line:
                    action, *rest = line[1:-2].split()
                    print ("LINE:", line, "ACTION:", action, "REST:", rest)
                    # if action == "put-down":
                    #    action == "put_down"
                    # elif action == "pick-up":
                    #     action = "pick_up"      
                    plan.append([action, list(map(lambda x: int(x), rest))])
        return plan
