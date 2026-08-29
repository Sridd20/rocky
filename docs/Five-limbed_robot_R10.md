LOCOMOTION ANALYSIS OF A MODULAR PENTAPEDAL
WALKING ROBOT

Cong Liu, Filippo Sanfilippo, Houxiang Zhang and Hans Petter Hildre
Faculty of Maritime Technology and Operation
Aalesund University College
Postboks 1517, N-6025 Aalesund, Norway

Chang Liu and Shusheng Bi
Robotics Institute, Beihang University,
Xueyuan Road 37, Haidian Dist., Beijing 100191, China

KEYWORDS
Pentapedal, Five-limbed, Gait.

ABSTRACT

In this paper, the configuration of a five-limbed modular
robot  is  introduced.  A  specialised  locomotion  gait  is
designed to allow for omni-directional mobility. Due to
the large diversity resulting from various gait sequences,
a  criteria  for  selecting  the  best  gaits  based  on  their
stability  characteristics
is  proposed.  A  series  of
simulations  is  then  performed  to  evaluate  the  various
gaits in different walking directions. A gait arrangement
scheme  toward  omni-directional  locomotion  is  finally
derived. Lastly, Experiments are also carried out on our
pentapedal  robot  prototype  in  order  to  validate  the
results of simulation. The  experiments confirm the gait
the
analysis  and  selection
evaluation of gait stability.

is  highly  accurate

in

I. INTRODUCTION

Legged  locomotion  offers  great  advantages  due  to  its
discrete  foothold  resulting  in  adaptability  to  uneven
terrain, low energy consumption and less environmental
destruction. Furthermore, the articulated limbs of legged
robots  are  able  to  function  in  both  locomotion  and
manipulation,  thereby  achieving  better  mobility  and
functionality.  Therefore,  a  robot  can  either  eliminate
deadlock by properly manipulating its limbs to regulate
the centre of gravity of its body, or improve its stability
by using an additional limb as a leg (Zhang et al. 1996).

So  far,  various  kinds  of  walking  machines  with  two,
four, six and eight legs have been developed. Systematic
studies have been conducted on gait generation, control
realization  and  algorithm  implementation.  However,
most of these studies were concentrated on models with
an even  number of legs, as research interests in an odd
number  of  legs  is  much  rarer.  Beside  the  research  in
(Zhang et al. 1996) and (Zhang et al. 1997), Prihastono
et  al.  proposed  a  five  legged  mechanism  that  was
inspired  by  a  starfish  (phylum  echynodermata).  The
robot  is  capable  of  performing  autonomous  navigation
in  cluttered  environments  (Prihastono  et  al.  2009).
However,  these  studies,  for  the  most  part,  considered

Proceedings 26th European Conference on Modelling and
Simulation ©ECMS Klaus G. Troitzsch, Michael Möhring,
Ulf Lotzmann (Editors)
ISBN: 978-0-9564944-4-3 / ISBN: 978-0-9564944-5-0 (CD)

only  fixed-shape  robots  that  were  designed  based  on  a
predefined and set-up  environment.

robots,  which

For  locomotion  in  unknown  and  hostile  environments,
modular  robot  platforms  are  expected  to  be  more
flexible and efficient than traditional fixed-shape robots,
since  such  situations  demand  on-site
locomotion
adaption,  shape  adaption  and  task  planning.  Moreover,
the  modular  approach  also  makes  the  mobile  robotic
system  versatile,  robust,  cost-effective  and  fast  to
prototype, so that new configurations of different robots
can  be  built  quickly  and  easily for  exploration.  By
differing  the  system  architecture,  modularity  can  be
recognised in several approaches. Ohira et al. developed
multi-legged  modular
be
interconnected  to  achieve  multiple  locomotion  modes
via  cooperation  and  accomplish  tasks  that  cannot  be
done  with  a  single  module  (Prihastono  et  al.  2009).  In
(Aoi  et  al.  2006),  Aoi  et  al.  presented  a  multi-legged
modular  robot,  consisting  of  six  homogenous  modules,
which  are  connected  to  each  other  via  a  3-degree-of-
freedom (3-DoF) joint. The leg joint is driven in such a
way that it follows the desired periodic trajectories, and
at the same time, acts as a passive shock absorber. Chen
et  al.  proposed  a  modular  method  for  formulating  the
dynamics  of  multi-legged  robots  with  general  leg
structures. In this approach, each leg is considered as an
individual module, while the whole multi-legged system
is  treated  as  a  free-floating  system  with  the  individual
leg module coupled to a main body (Chen et al. 1997).

can

Even though significant progress has been  made in this
field,  the  technology  of  multi-legged  robotics  is  still  a
challenging topic to be focused on in robotics research.
As  mentioned  above,  previous  research  work  either
focused  on  the  classic  four  and  six-limbed  modular
configurations,  or  on  fixed-shape  mobile  robots,  which
offered  insufficient  flexibility  in  locomotion.  In  this
paper,  by  combining
locomotion
robotic  approach,  a
techniques  with  a  modular
pentapedal  (five-limbed)  modular  robotic  configuration
is  proposed.  The  goal  of  this  research  is  to  develop  a
versatile  robotic  mobile  platform  featuring  an  easy-to-
locomotion
build  mechanical
capabilities and high manipulation flexibility.

structure,  various

the  multi-legged

II. RELATED WORK

A. Modular Robot Research

feature

robotic

robustness

building  blocks  with

systems
extendibility,

extraordinary
Modular
flexibility,
and
reconfigurability.  They  are  usually  composed  of
multiple
simple
repertoires,  together  with  uniform  docking  interfaces
which  enable  mechanical
junctions  and  electronic
communication  throughout  the  whole  system.  The  last
few  years  have  witnessed  an  increasing  interest  in
modular reconfigurable robotics for education (Daidié et
al.  2007),  inspired  robotic  research  (Zhang  et  al.  2007)
and space applications  (Yim et al. 2007).

relatively

is  suitable

robots  can  be  connected

in  various
Modular
configurations.  Chain-configuration
for
locomotion  and  manipulation  since  a  chain  of  modules
is equivalent to a leg or an arm. Such form is the focus
of  this  paper.  In  (González-Gómez  et  al.  2006),  1
dimension  (1D),  2D  and  3D  chain  robots  are  classified
according  to  their  topology.  A  1D-chain  robot,  such  a
snake  (Paap  et  al.  2000),  worm  (Zimmermann  et  al.
2004),  leg,  arm  or  cord  (Kurokawa  et  al.  2003),  can
transform  its  body  into  different  shapes,  enabling  itself
to pass though pipes,  grasp objects and  move on rough
terrain.

In  2004,  our  group  designed  a  series  of    low-cost
passive  modular  robots  and  modular  robot  locomotion.
In  cooperation  with  Dr.  Juan  González-Gómez,  the  Y1
modular robot with 1 DoF was designed in 2004 as the
first prototype (Gonzalez-Gomez et al. 2007). By using
this  prototype,
for
movement  of  snake-like  robots  were  studied  (Zhang  et
al.  2008).  Then  the  new  modular  robot  Cube-M  with
one DoF, an improved version of the Y1 modular robot,
was  presented  (Zhang  et  al.  2009).  It  features  an  easy-
to-build mechanical structure, four connecting faces, an
onboard  micro  controller  and  a  friendly  programming
environment.

the  minimal  configurations

B. The Pentapedal Configuration

found

focusing  on

To the best of the authors’ knowledge, limited literature
has  been
the  pentapedal
configuration  which  has  considerable  redundancy  in
locomotion  and  control.  However,  the  robot  could gain
auxiliary  competence  in  mobility  and  functionality  if  a
certain  proper  control  strategy  enables the  reuse  of  one
or  two  limbs  as  manipulator.  Our  goal  is  to  develop  a
pentapedal  robot  platform  capable  of  moving  on  rough
terrain and accomplishing tasks with redundant limbs.

The  pentapedal  configuration  has  no  corresponding
prototype  existing  in  nature,  though  some  quadrupedal
animals have certain auxiliary features to enhance their
locomotion.  For  example,  primates  utilise  their  tail  to
keep  their  balance  and  improve  climbing  performance.
In  addition,
implement  radial
symmetry  in the  robot’s general  configuration, inspired

is  advisable

to

it

by  pentamerism  in  nature.  Therefore,  the  pentapedal
robot  has
the  characteristics  of  omni-directional
locomotion.  However  in  special  cases,  it  is  able  to
transform into quadrupedalism while operating one limb
as a manipulator.

The  design  of  such  a  robotic  configuration  is  also
inspired  by  existing  multi-legged  robots.  The  omni-
directional  configuration  calls  for  high  equivalence
among  the  legs,  in  order  to  satisfy  the  accessibility  of
locomotion  in  various  directions.  On  each  leg  of  the
robot, the distribution of articulations adopts the classic
layout  of  insect-mimicking  robots.  The  three  Dof  are
provided by three articulation at the hip, knee and heel.

The control of a pentapedal robot also suggests a novel
gait to enable the robot to move in a fluent and efficient
manner.  Therefore  a  rational  gait  should  evenly  utilise
all  the  legs  while  maintaining  sufficient  stability.  This
paper  discusses  mainly  gait  generation  and  selection
towards walking stability and fluency.

Firstly  in  Section  III,  this  paper  outlines  the  sequential
3+2  gait,  which  is  specifically  designed  for  pentapedal
configuration.  Due  to  the  large  diversity  of  gait
sequences,  a
to
locomotion  stability  criteria  is  proposed  in  Section  IV.
In Section V, simulations are implemented to select the
best  gait  sequences.  Finally,  the  gaits  are  tested  by
experiments on our robot prototype.

selections  according

series  of

III. DEFINITION OF PENTAPEDAL
LOCOMOTION GAIT

Due  to  the  absence  of  similar  five-limbed  creatures  in
nature,  our  research  aims  to  introduce  a  fully  artificial
gait to be adopted to the robot.

the  static  stability  of

Considering
the  pentapedal
configuration, the robot  must  have a  minimum of three
supporting  feet  in  order  to  sustain  the  body.  The
remaining two limbs are utilised as striding legs in order
to  achieve  better  efficiency.  Therefore,  the  sequential
“3+2”  gait  for  the  pentapedal  robot  is  defined  as
follows:

a)  The gait cycle is divided into 5 phases. During
each  phase,  a  pair  of  feet  stride  one  pace’s
length in the walking direction of the robot.

b)  The striding foot-pair sequence is indicated by

the gait sequence.

c)

In  every  stride  phase,  the  striding  foot-pair
should not be two adjacent feet.

d)  The  of  robot’s  torso  movement  is  evenly
distributed  along  the  walking  direction  during
each  phase.  The  speed  is  therefore  at  0.4  time
of  pace  length  during  each  stride,  in  order  to
follow the movement of feet.

The  robot  walks  with  a  specified  sequence,  length  and
direction.  As  such,  the  trajectories  of  the  feet  are
determined.  In  Figure  1,  the  position  of  robot’s  torso
and feet are illustrated from left to right. The robot starts
the gait cycle from a neutral stance, as shown in the left-
most  picture,  and  then  strides  one  pair  of  feet  during
each state.  After  a  gait  cycle,  each  foot  has  been
involved  in  striding  twice.  The  robot  moves  twice  the
pace length after a whole gait cycle finishes.

account  in  the  analysis.  Hence  the  analysis  of    the
Sequential  “3+2”    Gait  is  based  on  each  state  during  a
gait  cycle.  Besides  the  5  phases  during  a  whole  gait
cycle, each phase is divided into a pre-stride state and a
post-stride  state  by  the  stride  movement.  Each  state
indicates a situation, recording the positions of all of the
feet as well as robot’s centre of gravity. Therefore, in 10
states  during  a  gait  cycle,  the  analysis  evaluates  each
gait  sequence  in  terms  of  its  stability,  as  shown  in
Figure 3.

The Sequential “3+2”  Gait provides an even utilization
of  striding  legs  while  maintaining  the  basic  stability  in
locomotion.  However,  by  presenting  a  sequence
permutation  between
the
number of various gaits reaches the factorial of 5, equals
to 120.

five  striding

foot-pairs,

Figure  1:  The  five  phases  during  a  Sequential  “3+2”
Gait  example.  The  robot  strides  its  legs  in  pairs
according  to  a  certain  sequence  while  the  torso  moves
towards the corresponding direction in fixed speed.

In order to clarify the  sequence of the striding feet, the
foot-pairs  are  numbered  1,  2,  3,  4  and  5,  as  shown  in
Figure 2. The pole is set at the centre of the body, while
the  polar  axis  is  defined  as  0°.  Due  to  the  radial
symmetric characteristic of the robot, the characteristics
in  the  range  of  [54°,126°),  [126°,198°),  [198°,270°),
[270°,342°)  and  [342°,54°)  are  identical.  In  addition,
these  ranges  are  axially  symmetric  with  respect  to  the
90°,  162°,  234°,  306°  and  18°  axes,  which  divide  their
own corresponding range into two symmetric parts. This
paper  only  focuses  on  the  direction  between  54°  and
90°, which can also represent situations in other ranges
by  using an appropriate transformation.

Figure 2: The Numbering of Feet and Foot-pairs

IV. LOCOMOTION GAIT ANALYSIS AND
SIMULATION

The  Sequential  “3+2”    Gait  can  secure  at  least  three
supporting feet during the gait cycle, but the robot may
eventually  fall  when  some  gaits  are  implemented.  The
goal of the gait analysis is to select best gaits to apply to
the robot.

Since the stride frequency never exceeds 3 Hz due to the
speed of the actuator, dynamic effects are not taken into

Figure  3:  In  each  phase  of  a  gait  cycle,  the  pre-stride
state  is  numbered  with  odd  numbers  while  the  post-
stride  state  with  even  numbers.  These  ten  states  are
sufficient  for  demonstrating  interaction  between  the
centre of gravity and supporting triangle.

Stability Analysis

The stability margin is defined as the minimum distance
from the projection of the centre of gravity to one of the
edges of the supporting triangle. When the projection of
the centre of gravity falls out of the supporting triangle,
the  robot  will  be  no  longer  stable  to  stand  on  the
supporting legs.

The  stability  margin  is  investigated  as  a  key  value  in
evaluating  the  stability  of  a  gait.  When  the  walking
direction x is  given,  a 120 × 10 matrix M(x), can  be
derived,  where M(x), indicates the stability  margin  of
sth state in No. n gait.

Two  indicies  are  designated  as  the  reference  of  the
stability  of  a  gait.  The  first  is  the  minimum  stability
margin M(x),  which  evaluates  the  most  vulnerable

situation  to  disturbances  during  the  whole  period.  The
to
summed
stability  margin  M(x)
investigate the overall stability of a gait.

 is  used

When the projection of the centre of gravity falls out of
the  supporting  triangle,  we  deem  that M(x), does  not
exist. Thus  the  value  of  M(x)  and M(x)  will
not be calculated due to its instability.

(),

(),

Figure 4: The Definition of Stability Margin

By  investigating  the  stability  margin  in  each  state,  any
gaits with unstable states are eliminated.

Simulation

A  series  of  parameters  derived  from  the  model  of  our
modular
is  adopted
pentapedal
correspondingly in the simulation.

robot  prototype

The robot’s five hips are placed at the five corners of a
pentagon  with a diameter of 188mm,  while the feet are
set similarly on a pentagon with a diameter of 340mm.
The  torso is  set  horizontally  at  a  height  of  90mm.  The
pace  length  for  each  stride  is  100mm,  as  shown  in
Figure  5.  Due  to  the  evenness  of  leg  placement,  the
robot’s  centre  of  gravity  is  supposed  to  coincide  with
the centre of the torso.

Figure 5: General Set-up of Simulation Model of
Pentapedal Robot

The  model  has  five  identical  legs  equally  distributed
around  the  torso.  Each  leg  consists  of  three  identical
modules,  providing  three  degrees  of  freedom  for
transition  movement.  The  kinematic  geometry  of  each
leg is shown as Figure 6.

Figure 6: The Kinematic Model of the Robot’s Leg

The stability margins of all states are calculated among
120  gaits.  The  calculation  is  performed  when  the
walking  direction  is  set  at  90°,  84°,  78°,  72°,  66°,  60°
and 54°. These 7 directions are selected evenly between
54° and 90° with an interval of 6°, sufficiently showing
the  distribution  of  superior  gaits  in  this  range  of
directions.

Stability in the 90° Walking Direction

When  the robot  moves in  the direction  of  90°, 52  gaits
keep their stability margins above 0 during all the states.
The  minimum  stability  margin  during  the  10  states  is
calculated as described in (1).

M(90) = MinM(90), 

           (1)

Table 1: List of Stable Gaits in 90° Walking Direction

Gaits with Specified Sequences

M(90) (mm)

No.79(2-3-4-5-1); No.94(2-1-5-4-3)
No.80(2-3-4-1-5); No.93(2-1-5-3-4)
No.83(2-3-1-5-4); No.92(2-1-3-4-5)
No.84(2-3-1-4-5); No.91(2-1-3-5-4)

No.49(3-4-5-2-1); No.119(1-5-4-2-3)
No.51(3-4-2-5-1); No.118(1-5-2-4-3)
No.52(3-4-2-1-5); No.117(1-5-2-3-4)
No.63(3-2-4-5-1); No.113(1-2-5-4-3)
No.64(3-2-4-1-5); No.114(1-2-5-3-4)
No.65(3-2-1-4-5); No.110(1-2-3-5-4)
No.66(3-2-1-5-4); No.109(1-2-3-4-5)

No.15(5-2-4-3-1); No.40(4-2-5-1-3)
No.16(5-2-4-1-3); No.39(4-2-5-3-1)
No.17(5-2-1-4-3); No.37(4-2-3-5-1)
No.18(5-2-1-3-4); No.38(4-2-3-1-5)
No.73(2-4-3-5-1); No.89(2-5-1-4-3)
No.74(2-4-3-1-5); No.90(2-5-1-3-4)
No.75(2-4-5-3-1); No.88(2-5-4-1-3)
No.76(2-4-5-1-3); No.87(2-5-4-3-1)

No.1(5-4-3-2-1); No.29(4-5-1-2-3)
No.3(5-4-2-3-1); No.28(4-5-2-1-3)
No.4(5-4-2-1-3); No.27(4-5-2-3-1)
No.5(5-4-1-2-3); No.25(4-5-3-2-1)
No.21(5-1-2-3-4); No.34(4-3-2-1-5)
No.22(5-1-2-4-3); No.33(4-3-2-5-1)
No.23(5-1-4-2-3); No.31(4-3-5-2-1)

20.21

12.58

4.54

4.03

Secondly,  the  sum  of  stability  margins  in  10  states  is
also  calculated  as  an  alternative  reference  to  evaluate
the stability of gaits as described in (2).

M(90) =  M(90),





           (2)

Table  I  shows  that  the  minimum  stab
gaits. All the stable gaits only appear to
20.21, 12.58, 4.54 and 4.03. At the same
are symmetric  with  respect  to  the  90°
have the same result since the direction
with the symmetry axis.

bility  margins  of
o have 4 values as
me time, gaits that
0°  axis  appear  to
n of 90° coincides

Gaits  whose  first  two  striding  foot-pa
appear to have the same  minimum stab
other  words,  the  selection  of  striding  f
first  two  phases  is  critical  to  the  total
whole cycle.

airs  are  identical
bility  margins. In
foot-pairs  in  the
al  stability  of  the

Figure 7 shows the total stability margin
the  52  gaits  in  the  blue  column,  to
minimum stability margins from Table
the  red  column.  The  gaits  with  a  h
stability margin also dominate in total st

in of 10 states for
ogether  with  the
e I represented by
higher  minimum
stability margin.

 (90)

     (90)

500
450

400

350

300

250
200

150

100

50
0

25

20

15

10

5

0

Total Stability Margin Minimum Stablit

lity Margin

Figure 7: Total and Minimum Stability
the Stable Gaits in the 90° Walking

y Margin of All
ng Direction

Stability in the Range between 54° and

nd 90°

Similar simulations are repeatedly carri
walking  direction  is  set  to  84°,  78°,  72
54°.

ried out while the
72°,  66°,  60°  and

28  gaits are  able  to  keep  their  stabilit
zero in these directions. The sets of top
these  walking  directions  still  show  grea
to the change of direction.

ity  margin  above
p-ranking gaits in
eat  similarity  due

The calculation of the stability margin i
widened  range.  Equations  (3)  and
minimum  and  total    stability  margin  of
walking directions.

is performed in a
d (4)  show
the
of  a  gait  in  the  7

 =

 

(90) , (84), (78)
(72), (66), (60)
(54)

,
,              (3)

,
= (90)  (84)
 (72)  (66)
 (54)

  (78)
  (60)

(4)

Figure  8  shows  the  gaits  w
stability margins and total stab

ability margins.

with  dominant  minimum

 

3300

3200

3100

3000

2900

2800

2700

2600

2500

 

24

22

20

18

16

14

12

10

Total Stability Margin

Minimum Stability Margin

Figure 8: Total and Minim
Dominant Stable Gaits in

mum Stability Margin of
n 7 Walking Directions

No.83(2-3-1-5-4), No.91(2-1-
4)  gaits  keep  their  minimum
20mm, and also keep high tot
all the situations.

-3-5-4) and No.93(2-1-5-3-
um  stability  margin  above
tal stability margins among

Gait Transformation for Om
Locomotion

mni-directional

After  investigating  the  stabili
range  between  54°  and  90°,
extraordinary  stability  emerge
analyzing the axial and radial
these three gaits can be transf
locomotion  in  directions  othe
54° to 90°.

lity  of  the  120  gaits in  the
,  three  superior gaits  with
ged  from  the  120  gaits.  By
l symmetric characteristics,
sformed appropriately to fit
her  than  the  range  between

According to the numbering in
4  is  respectively  symmetric  t
respect  of  90°  axis.  Therefor
the range between 90° and 12
5, 2-3-1-4-5 and 2-3-4-1-5.

in Figure 2, Foot-pair 1 and
to  Foot-pair  3  and  5 with
re,  the  three  best  gaits    in
26° are derived as 2-1-3-4-

Table 2: Optimised Gaits in

n All Walking Directions

Walking Direction
[54°,90°)
[90°,126°)
[126°,162°)
[162°,198°)
[198°,234°)
[234°,270°)
[270°,306°)
[306°,342°)
[342°,18°)
[18°,54°)

Selected Gaits

2-3-1
2-1-3
3-4-2
3-2-4
4-5-3
4-3-5
5-1-4
5-4-1
1-2-5
1-5-2

1-5-4; 2-1-3-5-4; 2-1-5-3-4
3-4-5; 2-3-1-4-5; 2-3-4-1-5
2-1-5; 3-2-4-1-5; 3-2-1-4-5
4-5-1; 3-4-2-5-1;3-4-5-2-1
3-2-1; 4-3-5-2-1; 4-3-2-5-1
5-1-2; 4-5-3-1-2; 4-5-1-3-2
4-3-2; 5-4-1-3-2; 5-4-3-1-2
1-2-3; 5-1-4-2-3; 5-1-2-4-3
5-4-3; 1-5-2-4-3; 1-5-4-2-3
2-3-4; 1-2-5-3-4; 1-2-3-5-4

Considering  the  radial  symmetric  characteristics,  the
corresponding  gaits  in  other  directions  can  also  be
derived by shifting the foot-pairs, as shown in Table 2.

V. EXPERIMENT

The  experiment  evaluating  the  sequential  “3+2”  gait  is
performed  on  our  pentapedal  robot.  In  fact,  all  of  the
geometric  parameters  and
in  our
simulation are inherited from this robot prototype.

initial  set-up

Figure 9: The pentapedal robot succeeds in walking in a
stable manner. The picture below shows gait No.79(2-3-
4-5-1) in comparison with the snapshots above.

During the experiments, the robot walks on a horizontal
even  plane,  while  it  performs  all  of  the  120  gaits.  The
gait  parameters  are  also  set  identical  to  the  simulation.
The walking direction is set to 90°.

Gait stability is judged based on observation. When the
robot does not stand on the supporting legs as planned,
or  any clash  of  legs  appears,  the  current  gait  will  be
deemed instable.

The  experimental  result  shows the 22  gaits  which  have
their minimum  stability  margin  equals  to  20.21  and
12.58  in  Table  I  to  have  satisfying  performance.  Other
the  ones  with  minimum  stability
gaits,
margins  below  10,  show  the  instability  in  some  phases
during a gait cycle.

including

The experiments also show that all these 22 stable gaits
do  not  appear  to have  any  clash between  legs.  In other
words, maintaining sufficient stability margin can, at the
same  time,  guarantee  a sparse lay-out  of  legs,  and thus
prevent the clash from occurring.

VI. CONCLUSION

The  modeling  and  simulation  has  greatly  reduced  the
work  of  testing  various  gaits,  and  provided  helpful
reference to judge the stability of gaits.

is  a
research
The  greatest  achievement  of  our
locomotion  control
toward  a  modular
strategy
pentapedal robot. First, the modular configuration of an
omni-directional  pentapedal  robot  was  introduced.  A
large  group  of  gaits  was  derived  after  the  Sequential
“3+2” Gait had been defined. Following this, a series of

analysis  and  simulation  were  performed  to  select  the
most  stable  gaits,  by  calculating  and  comparing  the
minimum  and
in  various
conditions. Lastly, experiments were carried out on our
robot prototype, giving evidence that high-ranking gaits
provide the robot with sufficient stability in locomotion.

total  stability  margins

Thus, the research on pentapedal robot is only confined
in  the  locomotion  on  horizontal  and  even  surface.  It  is
desirable  to  expand  this  method  to  accommodate  more
complicated  environment  in  future.  In  addition,  the
highly  pre-defined  gaits  deprive  the  possibility  which
enables  the  robot  to  decide  the  striding  strategy
intelligent
according
algorithm  with reversed mechanism  will be appreciated
to accomplish the gait generation.

the  current  situation.  An

to

REFERENCES

Aoi S, Sasaki H and Tsuchiya K. 2006. “Turning Maneuvers
of  a  Multi-legged  Modular  Robot  Using  Its  Inherent
Dynamic Characteristics”. In Proceedings of International
Conference on Intelligent Robots and Systems.IEEE. 180-
185.

Chen  W,  Yao  S  H  ,  Low  K  H  .  1997.  “Modular  formulation
for  dynamics  of  multi-legged  robots”.  In  Proceedings  of
8th  International  Conference  on  Advanced  Robotics
(Jul.7-9). 279-284.

Daidié  D,  Barbey  O  et  al..2007.  “The  DoF-Box  Project:  An
Educational  Kit  for  Configurable  Robots”.  Proceeding  of
2007(Zurich,
Advanced
Switzerland, Sep.4-7). 1-6.

Intelligent  Mechatronics

González-Gómez  J,  Zhang  H  et  al..2006.  “Locomotion
Capabilities  of  a  Modular  Robot  with  Eight  Pitch-Yaw-
Connecting  Modules”.  In  Proceeding  of   International
Conference
and  Walking  Robots
2006(Brussels, Belgium, Sep.12-14).

on  Climbing

Gonzalez-Gomez  J,  Zhang  H,  Boemo  E.  2007.  “Locomotion
Principles  of  1D  Topology  Pitch  and  Pitch-Yaw-
Connecting  Modular  Robots”,  In  Bioinspiration  and
Robotics:  Walking  and  Climbing  Robots  2007,  Maki  K.
Habib  (Ed.),  Advanced  Robotic  System  and  I-Tech
Education and Publishing. Vienna, Austria, 403-428.

Kurokawa H, Kamimura A, Yoshida E et al. 2003. “M-TRAN
II:  metamorphosis  from  a  four-legged  walker  to  a
caterpillar“. In Proceedings of International Conference on
Intelligent  Robots  and  Systems  2003(Las  Vegas,  USA,
Oct.27-31). IEEE.Vol.3, 2454-2459.

Paap K L, Christaller T, Kirchner F. 2000. “A robot snake to
inspect  broken  buildings”.  In  Proceeding  of  International
Conference  on  Intelligent  Robots  and  Systems  2000
(Takamatsu, Japan, Oct.30-Nov.5).IEEE. 2079-2082.

Prihastono,  Wicaksono  H.,  Anam  K.  et  al.  2009.
“Autonomous  five  legs  robot  navigation  in  cluttered
hybrid
environment  using
coordination  node”.  In  Proceeding  of  International  Joint

fuzzy  Q-learning

and

Web-page:
http://www.hials.no/eng/hials/research/mechatronics/people/fil
ippo_sanfilippo.

 HOUXIANG  ZHANG  received  Ph.D.  degree in
Mechanical  and  Electronic  Engineering  in  2003.  From
2004, he  worked as Postdoctoral Fellow at the Institute
of  Technical  Aspects  of  Multimodal  Systems  (TAMS),
Department  of  Informatics,  Faculty  of  Mathematics,
Informatics  and  Natural  Sciences,  University  of
Hamburg,  Germany.  Dr.  Zhang  joined  the  Department
of  Technology  and  Nautical  Sciences,  Aalesund
University College, Norway in April 2011 where he is a
Professor on Robotics and Cybernetics.
Email: hozh@hials.no.

HANS  PETTER  HILDRE  is  a  Professor  on  product
and system design at the Department of Technology and
Nautical  Sciences,  Aalesund  University  College,
Norway.
Email: hh@hials.no.

CHANG  LIU  is  PhD  candidate  Robotics  Institute,
Beihang University, Beijing.
Email: lcyituo0x01@163.com

SHUSHENG BI, born in 1966, is currently a professor
at  Robotics  Institute,  Beihang  University,  China.  He
received  his  PhD  degree  from  Beihang  University,
China,  in  2002.  His  research  interests  include  bionic
under  water  robots,  birdlike  robots,  and  flexible
microstructure design.
Email: biss_buaa@163.com.

Conference on Control, Automation and System. ICCAS-
SICE. (Fukuoka, Japan, Aug.18-21), 2871-2874.

Ohira  M,  Chatterjee  R,  Kamegawa  T  and  Matsuno  F.  2007.
“Development  of  Three-legged  Modular  Robots  and
Demonstration  of  Collaborative  Task  Execution”.  In
Proceeding  of  International  Conference  on  Robotics  and
Automation (Apr.10-14). IEEE, 3895-3900.

Yim  M,  Shen  W  et  al.  2007.  “Modular  Self-Reconfigurable
Robot Systems:  Challenges  and  Opportunities  for  the
Future”. IEEE Robotics & Automation Magazine, Vol.14,
No.1, March 2007. 2-11.

Zhang  H,  González-Gómez  J,  Chen  S  et  al.  2007.  “A  Novel
Modular Climbing  Caterpillar  Using  Low-frequency
Vibrating  Passive  Suckers”,  In  Proceeding  of  Advanced
Intelligent  Mechatronics  2007.  (Zurich,  Switzerland,
Sep.4-7). 1-6.

Zhang H, Gonzalez-Gomez J, Xie Z, Cheng S, Zhang J. 2008.
“Development of a Low-cost Flexible Modular Robot GZ-
I”.  In  Proceeding  of  2008  International  Conference  on
Advanced  Intelligent  Mechatronlics  (Xi'an,  China,  Jul.2-
5). 223-228.

Zhang  H,  Xie  Z,  González-Gómez  J,  Zhang  J.  2009.
“Embedded  Intelligent  Capability  of  a  Modular  Robotic
System”.  In  Proceeding  of  International  Conference  on
Robotics  and  Biomimetics  (Bangkok,  Thailand,  Feb.22-
25). IEEE. 2061 - 2066.

Zhang  J,  Jinsong  W  and  Bopeng  Z.  1996.  “The  locomotive
mode  study  on  five-limbed  robots”.  In  Proceedings  of
and
International  Conference  on  Systems,  Man
Cybernetics. IEEE, Vol. 2, 1595-1600.

Zhang J, Kiantiong Y and Jinsong W. 1997. “A multi-limbed
underwater  robot  and  its  gait  study”.  In  Proceedings  of
  Systems,  Man,  and
International  Conference  on
Cybernetics. IEEE, Vol.1, 755-760

Zimmermann K, Zeidis I, Steigenberger J. 2004. “On artificial
worms  as  chain  of  mass  points”.
In  Proceeding
of   International  Conference  on  Climbing  and  Walking
Robots 2004. 11-18.

AUTHOR BIOGRAPHIES

CONG LIU works in Aalesund University College as a
PhD  candidate  since  October  2011.  He  received  his
Electronics
Master’s
Engineering from Beihang University in Beijing.
Email: lico@hials.no.

in  Mechanical

diploma

FILIPPO SANFILIPPO was born in Catania, Italy and
obtained his Master’s Degree in  Computer Engineering
at    University  of  Siena  in  2011.  He  worked  for  six
months  at  Laboratory  of  Technical  Aspects  of
Multimodal  Systems
in  University  of  Hamburg,
investigating modularity in robotics.
Email: fisa@hials.no.

