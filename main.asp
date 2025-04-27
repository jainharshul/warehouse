% -------------------------------
% PARSING SECTION
% -------------------------------
% These rules count the total number of each object type based on init() facts.

% Number of Columns (X-axis size of grid)
gridWidth(GW):- GW=#count{X:init(object(node,I),value(at,pair(X,Y)))}.

% Number of Rows (Y-axis size of grid)
gridHeight(GH):- GH=#count{Y:init(object(node,I),value(at,pair(X,Y)))}.

% Total number of grid nodes
totalCells(TC):- TC=#count{I:init(object(node,I),value(at,pair(X,Y)))}.

% Total number of shelves
numShelves(TC):- TC=#count{I:init(object(shelf,I),value(at,pair(X,Y)))}.

% Total number of product types
numProducts(TC):- TC=#count{I:init(object(product,I),value(on,pair(X,Y)))}.

% Total number of picking stations
numPickingStation(TC):- TC=#count{I:init(object(pickingStation,I),value(at,pair(X,Y)))}.

% Total number of orders
numOrders(TC):- TC=#count{I:init(object(order,I),value(pickingStation,J))}.

% Total number of robots
numRobots(TC):- TC=#count{I:init(object(robot,I),value(at,pair(X,Y)))}.


% -------------------------------
% CONVERT INPUT SECTION
% -------------------------------
% These rules convert init() facts into predicates usable in the main logic.

% Define grid coordinates aTC node labels
nodeAt(TCI,pair(X,Y)):- init(object(node,TCI),value(at,pair(X,Y))).
pair(X,Y):- init(object(node,TCI),value(at,pair(X,Y))).
node(TCI):- init(object(node,TCI),value(at,pair(X,Y))).

% Define highway cells
highway(TCI):- init(object(highway,TCI),value(at,pair(X,Y))).

% Define picking stations aTC their node positions
pickingStationAt(PSI,TCI):- init(object(pickingStation,PSI),value(at,pair(X,Y))), init(object(node,TCI),value(at,pair(X,Y))).
pickingStation(PSI):- init(object(pickingStation,PSI),value(at,pair(X,Y))), init(object(node,TCI),value(at,pair(X,Y))).

% Define initial robot positions
botLoc(RI,object(node,TC),0):- init(object(robot,RI),value(at,pair(X,Y))), nodeAt(TC,pair(X,Y)).
robot(RI):- init(object(robot,RI),value(at,pair(X,Y))).

% Define initial shelf positions
shelfAt(SI,object(node,TC),0):- init(object(shelf,SI),value(at,pair(X,Y))), nodeAt(TC,pair(X,Y)).
shelf(SI):- init(object(shelf,SI),value(at,pair(X,Y))).

% Define products aTC quantities on shelves
productOn(PRI,object(shelf,SI),with(quantity,PQ),0):- init(object(product,PRI),value(on,pair(SI,PQ))).
product(PRI):- init(object(product,PRI),value(on,pair(SI,PQ))).

% Define orders: location (picking station) aTC product requirements
order(OI,object(node,TC),contains(PRI,PQ),0):- init(object(order,OI),value(pickingStation,PKI)), pickingStationAt(PKI,TC), init(object(order,OI),value(line,pair(PRI,PQ))).
order(OI):- init(object(order,OI),value(pickingStation,PKI)).


% -------------------------------
% MAIN LOGIC SECTION
% -------------------------------
% Main program logic: actions, transitions, constraints, aTC optimization.

#const n=50.  % Number of time steps

% Define allowed move directions
move(0,1;0,-1;-1,0;1,0).

% -------------------
% ACTION CHOICE RULES
% -------------------
% At each time step, each robot may optionally perform one of the following actions.

{robotMove(R,move(DX,DY),T):move(DX,DY)}1:- R=1..GW, numRobots(GW), T=0..TN,TN=n-1.
{liftShelf(R,SI,T):shelf(SI)}1:- R=1..GW, numRobots(GW), T=0..TN,TN=n-1.
{dropShelf(R,SI,T):shelf(SI)}1:- R=1..GW, numRobots(GW), T=0..TN,TN=n-1.
{fulfill(R,OI,with(SI,PR,DQ),T):order(OI,object(node,TC),contains(PR,OQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), DQ=1..PQ}1:- R=1..GW, numRobots(GW), T=0..TN,TN=n-1.

% -------------------
% ACTION TO happen MAPPING
% -------------------
happen(object(robot,R),move(DX,DY),T):-robotMove(R,move(DX,DY),T).
happen(object(robot,R),pickup,T):-liftShelf(R,_,T).
happen(object(robot,R),putdown,T):-dropShelf(R,_,T).
happen(object(robot,R),fulfill(OI,PRI,DQ),T):-fulfill(R,OI,with(SI,PRI,DQ),T).

% Robots can’t do more than one action per time step
:- happen(object(robot,R),A1,T), happen(object(robot,R),A2,T), A1!=A2.

% -------------------
% MOVEMENT CONSTRAINTS
% -------------------
% Prevent robots from moving out of bouTCs
:- botLoc(RI,object(node,TC),T), robotMove(R,move(DX,DY),T), nodeAt(TC,pair(X,Y)), X+DX<1.
:- botLoc(RI,object(node,TC),T), robotMove(R,move(DX,DY),T), nodeAt(TC,pair(X,Y)), Y+DY<1.
:- botLoc(RI,object(node,TC),T), robotMove(R,move(DX,DY),T), nodeAt(TC,pair(X,Y)), X+DX>GH, numColumns(GH).
:- botLoc(RI,object(node,TC),T), robotMove(R,move(DX,DY),T), nodeAt(TC,pair(X,Y)), Y+DY>GW, numRows(GW).

% -------------------
% PICKUP/PUTDOWN CONSTRAINTS
% -------------------
:- 2{liftShelf(R,S,T): robot(R)}, shelf(S).  % No shelf picked up by multiple robots
:- liftShelf(RI,S1,T), shelfAt(S2,object(robot,RI),T).  % Already carrying
:- liftShelf(R1,S,T), shelfAt(S,object(robot,R2),T).  % Already picked up
:- liftShelf(RI,S,T), shelfAt(S,object(node,TC),T), not botLoc(RI,object(node,TC),T).  % Must be co-located

:- 2{dropShelf(R,S,T): robot(R)}, shelf(S).
:- dropShelf(RI,S,T), not shelfAt(S,object(robot,RI),T).
:- dropShelf(RI,S,T), botLoc(RI,object(node,TC),T), highway(TC).  % Can't drop on highway

% -------------------
% fulfillY CONSTRAINTS
% -------------------
:- fulfill(R,OI,with(_,PR,_),T), order(OI,object(node,TC),contains(PR,_),T), not botLoc(R,object(node, TC),T).
:- fulfill(R,OI,with(SI,PR,_),T), productOn(PR,object(shelf,SI),with(quantity,_),T), not shelfAt(SI,object(robot,R),T).
:- fulfill(R,OI,with(SI,PR,DQ),T), order(OI,object(node,TC),contains(PR,OQ),T), DQ>OQ.
:- fulfill(R,OI,with(SI,PR,DQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), DQ>PQ.

% -------------------
% HIGHWAY CONSTRAINTS
% -------------------
:- pickingStationAt(_,TCI), highway(TCI).
:- shelfAt(S,object(node,TCI),_), highway(TCI).

% -------------------
% COLLISION / SWAP / DUPLICATION CONSTRAINTS
% -------------------
% Robots can't share a cell
:- 2{botLoc(R,object(node,TC),T):node(TC)}, robot(R), T=0..n.
:- 2{botLoc(R,object(node,TC),T):robot(R)}, node(TC), T=0..n.

% Prevent robot swap
:- botLoc(R1,object(node,TC1),T), botLoc(R1,object(node,TC2),T+1), botLoc(R2,object(node,TC2),T), botLoc(R2,object(node,TC1),T+1), R1!=R2.

% Prevent multiple shelf locations or carriers
:- 2{shelfAt(S,object(robot,GW),T): robot(GW)}, shelf(S), T=0..n.
:- 2{shelfAt(S,object(robot,GW),T): shelf(S)}, robot(GW), T=0..n.
:- 2{shelfAt(S,object(node,TC),T): node(TC)}, shelf(S), T=0..n.
:- 2{shelfAt(S,object(node,TC),T): shelf(S)}, node(TC), T=0..n.
:- shelfAt(S,object(node,_),T), shelfAt(S,object(robot,_),T).

% -------------------
% STATE TRANSITIONS
% -------------------
botLoc(R,object(node,NEW_TC),T+1):- botLoc(R,object(node,TC),T), nodeAt(TC,pair(X,Y)), nodeAt(NEW_TC, pair(X+DX,Y+DY)), robotMove(R,move(DX,DY),T).
shelfAt(S,object(robot,RI),T+1):- liftShelf(RI,S,T), shelfAt(S,object(node,TC),T), botLoc(RI,object(node,TC),T).
shelfAt(S,object(node,TC),T+1):- dropShelf(RI,S,T), shelfAt(S,object(robot,RI),T), botLoc(RI,object(node,TC),T).
order(OI,object(node,TC),contains(PR,OU-DQ),T+1):- fulfill(R,OI,with(SI,PR,DQ),T), order(OI,object(node,TC),contains(PR,OU),T).
productOn(PR,object(shelf,SI),with(quantity,PQ-DQ),T+1):- fulfill(R,OI,with(SI,PR,DQ),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T).

% -------------------
% INERTIA (Frame Axioms)
% -------------------
botLoc(R,object(node,TC),T+1):- botLoc(R,object(node,TC),T), not robotMove(R,move(_,_),T), T<n.
shelfAt(S,object(node,TC),T+1):-shelfAt(S,object(node,TC),T), not liftShelf(_,S,T), T<n.
shelfAt(S,object(robot,RI),T+1):-shelfAt(S,object(robot,RI),T), not dropShelf(RI,S,T), T<n.
order(OI,object(node,TC),contains(PR,OU),T+1):- order(OI,object(node,TC),contains(PR,OU),T), productOn(PR,object(shelf,SI),with(quantity,PQ),T), not fulfill(_,OI,with(SI,PR,_),T), T<n.
productOn(PR,object(shelf,SI),with(quantity,PQ),T+1):- productOn(PR,object(shelf,SI),with(quantity,PQ),T), not fulfill(_,_,with(SI,PR,_),T), T<n.

% -------------------
% FINAL GOAL CHECK
% -------------------
:- not order(OI,object(node,_),contains(PR,0),n), order(OI,object(node,_),contains(PR,_),0).

% -------------------
% OPTIMIZATION
% -------------------
numActions(N):-N=#sum{1,O,A,T:happen(O,A,T)}.
timeTaken(N-1):-N=#count{T:happen(O,A,T)}.
#minimize{1,O,A,T:happen(O,A,T)}.         % Minimize number of actions
#minimize{T:happen(O,A,T)}.              % Minimize makespan

% -------------------
% OUTPUT
% -------------------
#show happen/3.
#show numActions/1.
#show timeTaken/1.