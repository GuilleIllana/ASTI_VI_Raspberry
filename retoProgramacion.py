"""-----------------------------------------
Desafio programacion ASTI ROBOTICS 2019/2020

Version: 1.1
-------------------------------------------"""

import numpy as np
import pylab as pl


#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#    NO TOCAR
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

class Robot:
    def __init__(self,Tfin):
        self.__x = 0
        self.__y = 1
        self.__ang= np.pi/4
        self.__inct=0.1
        self.__t=0
        self.__vizq=0
        self.__vder=0
        self.__l=0.2
        self.__final=False
        self.__dfinal=1
        self.__fasesCompletadas=0
        self.__tTotal=0
        self.__tiempos=list()
        
        self.__xlist=list()
        self.__ylist=list()
        self.__anglist=list()
        self.__psenlist=list()
                
        self.__fase=-1
        self.__numFases=9
        self.__tiempos=[Tfin]*self.__numFases      
        
        self.__posSensor=[self.__x+(self.__l/2)*np.cos(self.__ang),self.__y+(self.__l/2)*np.sin(self.__ang)]
               
        self.__rectas=list([[0,10],[0,10],[0,10],[0,10],[0,10],[1,0],[0,10],[0,10],[1,0]])
        self.__destinos=list([[20,10],[20,10],[20,10],[20,10],[20,10],[20,20],[20,10],[20,10],[20,20]])
        self.__origenes=list([[0,10,0],[0,10,np.pi/4],[0,20,0],[0,0,np.pi/4],[0,0,-np.pi/4],[0,10,-np.pi/4],[0,10,np.pi],[0,20,np.pi],[0,10,3*np.pi/4]])
        
        self.__nuevaFase()
        
    def actTiempo(self):
        def calcDistSensor(puntoRecta,posEsquinaDer,posEsquinaIzq,posSensor):
            dist1=np.sqrt(pow(puntoRecta[0]-posEsquinaDer[0], 2)+pow(puntoRecta[1]-posEsquinaDer[1],2))
            dist2=np.sqrt(pow(puntoRecta[0]-posEsquinaIzq[0], 2)+pow(puntoRecta[1]-posEsquinaIzq[1],2))
            if dist1 < dist2:
                return np.sqrt(pow(puntoRecta[0]-posSensor[0], 2)+pow(puntoRecta[1]-posSensor[1],2))
            else:
                return -np.sqrt(pow(puntoRecta[0]-posSensor[0], 2)+pow(puntoRecta[1]-posSensor[1],2))
                
        if self.__final==True:
            if self.__fase < self.__numFases-1:
                self.__nuevaFase()
            else:
                return
            
        self.__t+=self.__inct
        self.__x += 0.5*(self.__vizq+self.__vder)*np.cos(self.__ang)*self.__inct
        self.__y += 0.5*(self.__vizq+self.__vder)*np.sin(self.__ang)*self.__inct
        self.__ang+=(1/self.__l)*(self.__vder-self.__vizq)*self.__inct 
        self.__distDest=np.sqrt(pow(self.__destino[0]-self.__x, 2)+pow(self.__destino[1]-self.__y,2))
        if self.__distDest< self.__dfinal:
            self.__fasesCompletadas+=1
            self.__tiempos[self.__fase]=self.__t
            self.__tTotal+=self.__t
            self.__final=True
        
        #Sensor            
        self.__posSensor=[self.__x+(self.__l/2)*np.cos(self.__ang),self.__y+(self.__l/2)*np.sin(self.__ang)]
        
        posEsquinaDer=[self.__posSensor[0]+(self.__l/2)*np.cos(self.__ang-np.pi/2), self.__posSensor[1]+(self.__l/2)*np.sin(self.__ang-np.pi/2)]        
        posEsquinaIzq=[self.__posSensor[0]+(self.__l/2)*np.cos(self.__ang+np.pi/2), self.__posSensor[1]+(self.__l/2)*np.sin(self.__ang+np.pi/2)]        
        
        m=(self.__posSensor[1]-posEsquinaDer[1])/(self.__posSensor[0]-posEsquinaDer[0])
        
        if np.isinf(m):
            b=self.__posSensor[0]
        else:
            b=self.__posSensor[1]-m*self.__posSensor[0]
        rectaSensor=[m,b]
         
        self.__distSensor=-1
        if m==self.__mRecta : #paralelas
            if b==self.__bRecta:
                self.__distSensor=0
            else:
                self.__distSensor=np.nan
        else:
            if m is np.nan or np.isinf(m):
                if self.__mRecta is np.nan or np.isinf(self.__mRecta):
                    self.__distSensor=np.nan
                else:
                    puntoX=b
                    puntoY=self.__mRecta*puntoX+self.__bRecta
            else:
                if self.__mRecta is np.nan or np.isinf(self.__mRecta):
                    puntoX=self.__bRecta
                    puntoY=m*puntoX+b
                else: 
                     puntoX=(self.__bRecta-b)/(m-self.__mRecta)
                     puntoY=m*puntoX + b
        
        if self.__distSensor ==-1 :
            self.__distSensor=calcDistSensor([puntoX,puntoY],posEsquinaDer, posEsquinaIzq,self.__posSensor)
                
        self.__xlist[self.__fase].append(self.__x)
        self.__ylist[self.__fase].append(self.__y)
        self.__anglist[self.__fase].append(self.__ang)
        self.__psenlist.append(self.__distSensor)
        
    def leerT(self):
        return self.__t
    
    def leerFinFases(self):
            if self.__fase < self.__numFases-1:
                return False
            else:
                return True
            
    def leerFasesSuperadas(self):
            return self.__fasesCompletadas

    def leerTiempoTotal(self):
            return self.__tTotal
        
    def leerTiempoFase(self, fase):
            return self.__tiempos[fase]
        
    def leerFinal(self):
        return self.__final
    
    def leerDistSensor(self):
        return self.__distSensor
    
    def leerDistDest(self):
        return self.__distDest
     
    def fijarVel(self,vIzq,vDer):
        if vIzq > 2:
            self.__vizq=2
        elif vIzq <-2:
            self.__vizq=-2
        else:
            self.__vizq=vIzq                
        if vDer > 2:
            self.__vder=2
        elif vDer < -2:
            self.__vder=-2
        else:
            self.__vder=vDer
        
    def __str__(self):
        return str([self.__x, self.__y, self.__ang])
    
    def __nuevaFase(self):
        self.__fase+=1
        self.__t=0
        self.__mRecta=self.__rectas[self.__fase][0]
        self.__bRecta=self.__rectas[self.__fase][1]
        self.__x =self.__origenes[self.__fase][0]
        self.__y = self.__origenes[self.__fase][1]
        self.__ang = self.__origenes[self.__fase][2]
        self.__xlist.append([])
        self.__ylist.append([])
        self.__anglist.append([])
                
        self.__destino=self.__destinos[self.__fase]
        self.__distDest=np.sqrt(pow(self.__destino[0]-self.__x, 2)+pow(self.__destino[1]-self.__y,2))
        self.__final=False
        
    def pintar(self):
        if self.__fasesCompletadas==self.__numFases:
            n=self.__fasesCompletadas
        else:
            n=self.__fasesCompletadas+1
            
        for i in range(n):  
            fig=pl.figure(figsize=(8, 6), dpi=80)            
            if self.__rectas[i][0] is np.nan or np.isinf(self.__rectas[i][0]):
                y = np.linspace(np.amin([np.amin(self.__ylist[i]),self.__destinos[i][1]]), np.amax([np.amax(self.__ylist[i]),self.__destinos[i][1]]), 1000)
                pl.plot([self.__rectas[i][1]]*1000, y, color="blue", linewidth=1.0, linestyle="-", label='Linea de guia')    
            else:
                x = np.linspace(np.amin([np.amin(self.__xlist[i]),self.__destinos[i][0]]), np.amax([np.amax(self.__xlist[i]),self.__destinos[i][0]]), 1000)
                pl.plot(x, x*self.__rectas[i][0]+self.__rectas[i][1], color="blue", linewidth=1.0, linestyle="-",label='Linea de guia')    
            #pl.hold(True)
            #pl.plot(self.__xlist[i], self.__ylist[i], color="blue", linewidth=1.0, linestyle="-")    
            pl.quiver(self.__xlist[i], self.__ylist[i],np.cos(self.__anglist[i]),np.sin(self.__anglist[i]),label='Robot')
            pl.plot(self.__destinos[i][0],self.__destinos[i][1],'o',color="red", markersize=5,label='Destino')
            fig.suptitle('Fase: ' + str(i) + ' Tiempo: ' + str(self.__tiempos[i]), fontsize=16)
            pl.plot(self.__origenes[i][0],self.__origenes[i][1],'o',color="green", markersize=5,label='Origen')
            pl.legend(loc='upper left')
            
    
#----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

    
#PROGRAMA PRINCIPAL
#----------------------    
Tfin=200  
r=Robot(Tfin)

while r.leerT() < Tfin and not r.leerFinFases():
    r.actTiempo()
    Tfin=200
    
    #------------------------------------
    # Tocar a partir de aqui {
    #----------------------------------
   
    
    #------------------------------------
    # } No tocar desde aqui
    #----------------------------------
    
    while r.leerT() < Tfin  and not r.leerFinal():
        r.actTiempo()
        
        #------------------------------------
        # Tocar a partir de aqui {
        #----------------------------------
    
        
        
        
        
        
        #------------------------------------
        # } No tocar desde aqui
        #----------------------------------
      
#-----------------

if  r.leerFasesSuperadas()==9:
    print( "--------------------")
    print (" :) Ha llegado al final")
    print( "---------------------")
    print( "Fases superadas:")
    print( r.leerFasesSuperadas())   
    for i in range(r.leerFasesSuperadas()):
        print(["Fase: ", i, "Tiempo: ", r.leerTiempoFase(i)])
    print( "Tiempo total: ")
    print( r.leerTiempoTotal())
else:
    print( "--------------------")
    print( " :(  Intentelo de nuevo")
    print("---------------------")
    print( "Fases superadas:")
    print( r.leerFasesSuperadas() )
    print( "Tiempo por fase:")
    for i in range(r.leerFasesSuperadas()+1):
        print(["Fase: ", i, "Tiempo: ", r.leerTiempoFase(i)])
    print( "Tiempo total: ")
    print( r.leerTiempoTotal()+Tfin)
    
#DIBUJO
r.pintar()
