## Combining myoleg model with the exoskeleton model

### Description for each document:

(1) **amber_sim:** exoskeleton model

(2) **myo_sim:** muscle model myoleg

(3) **emptyexo.xml:** xml file for only exoskeleton model without human body

(4) **emptywmuscle.xml:** xml file for exoskeleton model with muscle model, connecting from exoskeleton to muscle, without human body, limited tendon length 

(5) **generateemptywm.py:** python script for exoskeleton model with muscle model, without human body, limited tendon length

(6) **generate_mujoco_model.xml:** generated script from python script for exoskeleton model with muscle model, without human body, limited tendon length

(6) **exomujoco.py:** python script for exoskeleton model with muscle model, with human body, no limit on tendon length

(7) **ExoMuscle.xml:** first edition of muscle and exoskeleton combination

(8) **exotendon.xml:** manually adding four tendons on each front and back of the hip and knee of the left and right leg

(9) **exowmuscle.xml:** xml file for exoskeleton model with muscle model, connecting from the muscle model to exoskeleton model, with human body, unlimited tendon length 

(10) **testversion.py:** draft file for python script

(11) **MJDATA.TXT:** data from mujoco model

(12) **JOINTAngle.py:** calculate the joint angle from the TXT file and calculate the angle difference between the selected joints in muscle model and exoskeleton mode
 
(13) **JointAngleRMSE.py:** calculate the joint angle from the TXT file and calculate the RMSE between the selected joints pairs in muscle model and exoskeleton mode