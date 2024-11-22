 #connexion avec VREP
PATHCSIM="P:/mea5/Modelisation & commande avancée/TP";
include("P:/mea5/Modelisation & commande avancée/TP/lib-robotique.jl"); 
CreateRobotKukaLwr()
include("P:/mea5/Modelisation & commande avancée/TP/lib-CSim.jl")

global clientID=startsimulation(simx_opmode_oneshot) # On lance une instance de connexion avec VREP

if clientID==0 println("Connexion établie")
    else println("Erreur de connexion")
end


T7=zeros(4,4);
init_pos()

global rob=CreateRobotKukaLwr();
global θinit=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
global pA=[-0.3668, -0.0379, 0.8634];

# Robot en position initiale
setjointposition(clientID,θinit,7,0,objectname_kuka)
sleep(2)

#Implémentation de la tâche A

# Paramètres de la tâche
global pD = [-0.3668, -0.0379, 0.5634];  # Position désirée (mouvement en Z)
global dt = 0.01;  # Pas de temps
global err_ts = 0.01;  # Tolérance d'arrêt
global Pe = pA;  # Position actuelle du bout du robot

# Générer les configurations articulaires avec MCI
Q = MCI(rob, θinit, pD, dt, err_ts);
#println("Configurations générées par MCI : ", Q)
N = Int(length(Q)/7);
println("N = ", N);

for i=1:N
    s = (7*i)-6;
    e = (7*(i+1))-7;
    #print("Angle n°",i);
    #println(Q[s:e]);
    setjointposition(clientID,Q[s:e],7,0,objectname_kuka)
    sleep(dt)
end


#print("pos=",getjointposition(clientID,7,0,objectname_kuka))

stopsimulation(clientID,simx_opmode_oneshot) # Arrêt de la simulation


