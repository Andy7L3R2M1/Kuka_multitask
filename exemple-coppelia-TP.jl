 #connexion avec VREP
PATHCSIM="C:/Users/Acer/Documents/Etude/Année 2023-2024/Polytech/MEA5/modelisation et cmd 3d/TP1-Modélisation avancée- hierarchie des taches/Kuka_multitask/";
include("C:/Users/Acer/Documents/Etude/Année 2023-2024/Polytech/MEA5/modelisation et cmd 3d/TP1-Modélisation avancée- hierarchie des taches/Kuka_multitask/lib-robotique.jl"); 
CreateRobotKukaLwr()
include("C:/Users/Acer/Documents/Etude/Année 2023-2024/Polytech/MEA5/modelisation et cmd 3d/TP1-Modélisation avancée- hierarchie des taches/Kuka_multitask/lib-CSim.jl")

#Partie établissement commmunication avec le serveur-modele simulation du bras 7 DDL de Coppelia_Sim-------------------------------------------------------------------------------------------
global clientID=startsimulation(simx_opmode_oneshot) # On lance une instance de connexion avec VREP

if clientID==0 println("Connexion établie")
    else println("Erreur de connexion")
end

#Parametre global du robot sur Coppélia_Sim
global robot=CreateRobotKukaLwr();
θinit=[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
pA=[-0.3668, -0.0379, 0.8634];
T7=zeros(4,4);

# Paramètres de la Tâche A
#global pD = [-0.3668, -0.0379, 0.5634];  # Position désirée (mouvement en Z)
dt = 0.01; # Pas de temps
err_ts = 0.001;  # Tolérance d'arrêt
zB = 0.5;

# Robot en position initiale
init_pos()
sleep(2)

#Appel Tâche_1 :
#cmd_translation_z(robot, θinit, pA, zB, dt, err_ts);

#Appel Tâche_2 :

#Reset des positions
print("pos=",getjointposition(clientID,7,0,objectname_kuka))

#Arret de la simulation
stopsimulation(clientID,simx_opmode_oneshot) # Arrêt de la simulation


