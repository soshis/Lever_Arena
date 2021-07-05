****************************************************************************************************************
**************************************  Lever task arena protocol **********************************************


Adrien Boissenin
Last modifictation:5/29/2017



*********************** Procedure *********************** 

Sessions last from 5 to 20 min. Animals are fully water restricted for 24 hours a day except 30 min to an hour of 
ad libitum water following the completion of the training session for the day.


*****Pre-training (4-6 days):

Acclimatization period

- Step 1: 3 days after animal arrive at the facility, start handling for 5-10 min for each for 2-3 days.

- Step 2: introduce treats and apple juice with handling animals (use the staircase to imitate the lever arena) for 
2-3 days.


After the one-week acclimatization period, begin water restriction by 4-hour increments per day until 24 hours’ 
restriction is achieved. Body weight is monitored. At the same time, handling and introduction to each reward is 
performed.

****** Training (15-20 days):

Train animals according to the protocol detailed below. Training sessions last approximately 10 min to 15 min and 
animal received a reward when they nose poke and push joystick simultaneously, (exceptions will be explicit in the 
step descriptions). Ideally, rats are trained 5 consecutive days per week, during the two days without training they 
have access to ad libitum water.

Use training python script:
Open 'Training_Step(number).py file' in lever arena folder -> press F5 (should make the program start) -> type in 
animal’s number -> start training -> After training press Ctrl+C to terminate session.

Exception for Step6: Open “Training_Step(number).py' file in lever arena folder -> press F5 (should make the 
program start) -> type in animal’s number -> type last vertical position reached (0, 1 or 2) -> type last horizontal 
position reached (0, 1, 2, 3 or 4) -> start training -> After training press Ctrl+C to terminate session.


*** Phase 1 : nose spoke, lever push.

- Step 1: perform nose poke. Success for nose poke is notified with beep sound and reward. After a single session 
	with 40 trials, the step is completed. Don’t forget to progressively adjust the gain knob to its minimum. Nose 
	spoke should not be neglected. Rats that don't learn to do nose spoke right inside the sensor neck performed 
	poorer when the task become more complicated. They learn slower as they get easier confuse and their lever pushes 
	are less ‘proper’(expected time: 2 sessions).

- Step 2: Learning to push the lever. Each time, start with 5 nose spokes to get the reward. First real lever is in low 
	position (v_pos = 0.00110, h_pos =0 .0130). Use dummy lever to draw rat’s attention. Give reward when rat touches 
	the dummy lever and then when the rat touches the dummy lever. Start by putting the dummy lever at a similar level 
	than the platform and then, to prevent him using its teeth, lower your dummy lever. Once rat learned to pushes a 
	little bit the dummy lever, shift attention from dummy lever to real lever by rising the real lever to 
	h_pos = 0.00120 and finally h_pos = 0.00130 (threshold = 2500). The script will stop and ask if you want to rise 
	the lever after the 10th, 20th and 30th successes. press 'y' (yes) or 'n' (no) then press 'enter' to move it 
	-or not-. Once you believe rats acquire the movement step is complete. (expected time: 2 sessions).

- Step 3: Learning to push the lever on a wider range with an adaptive threshold. Joystick is moved at v_pos = 0.00130 and 
	h_pos = 0.00120 horizontal position (threshold set at 2400). Success for the trial is notified with beep sound. 
	Lever threshold increases each 5 corrects push. If 5 incorrect pushes on the Lever before a successful trial, the 
	lever threshold decrease. When the rat reached a threshold of at least 3100 (maximum threshold is 3200), the step 
	is complete. (expected time: 1 sessions or 2 sessions, can be done right after step2 if training time remains).

- Step 4: Association nose spoke and lever push: Begin with 10 nose spokes-only with a lever at an unreachable position 
	(v_pos = 0.00110). Then, lever is moved: v_pos = 0.00130 and h_pos = 0.00120, threshold= 2400. Usually, the rat will 
	push the lever, maintaining it pushed while exploring the environment with its nose. once he learns the association: 
	push + nose spoke => reward, he will become more efficient (doing nose spoke, then a short push on the lever). Again 
	don’t underestimate the importance of the step. The rat need to stick is head right in front of the sensor. Rat 
	performing well here, usually learn quicker the most complex step as they can focus on the lever push. When you 
	noticed the rat learn to do both tasks to get the reward, go step 5. (expected time: 2 sessions).


***Phase 2: Maximum lever amplitude at every position of the lever.

- Step 5: Last and longest step. In this step, the lever is set at the initial position (v_pos = 0.00130 and h_pos = 0.00105)
	and will move to every position row after row. The rat is expected to push the lever, beyond the threshold of 3200, 
	4 times before moving to the next position. An adaptive threshold is setup to assist the rat in that task. First, 
	4 incorrect push allow to reduce the threshold, the number is low to keep the rats motivated to do a large number of 
	pushes. However, for the last row, it was observed that the rats performed better when the number of pushes allowing to 
	reduce the threshold is set to 7 or 8. The task is considered complete when the rats reach maximum threshold (3200) 4 
	times at row 2 and column 4 (most remote position). (expected time: as long as needed, 4-8 sessions).

- Step 6: Last step. Starting from row 0, column 0, the goal is to go through each position 2 times (aka 90 successes). After 
	three corrects push the lever will move (threshold at 3200). Once complete the training in over. I observed that to be
	completed the first time the rat need at least 20 minutes but this time is then reduced to 15 min the next session 
	(expected time: 1 or 2 sessions).

***Phase 3: After the training

- Step Random: Once the animals are trained, you can reduced the training to 2 or 3 5min-session per week. At this step, the 
	lever move to a random position (among the 15 learned) every 3 correct pushes). Usually maintain the training until you
	stop the water restriction before the surgery.
