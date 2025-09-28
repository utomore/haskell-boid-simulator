module Main (main) where

import Lib

import qualified Graphics.Gloss as GL
import Graphics.Gloss.Interface.Pure.Simulate
import Linear.V2 (V2(..)) 
import Linear.Vector ((^*), (^/))
import Linear.Metric (quadrance, norm, Metric (norm), normalize)
import System.Random (mkStdGen, Random(randomRs), splitGen)

newtype Position = Position (V2 Float) deriving (Show)

newtype Velocity = Velocity (V2 Float) deriving (Show)

newtype Acceleration = Acceleration (V2 Float) deriving (Show)

data Boid = Boid {
    boidId :: Int,
    boidPos :: Position,
    boidVel :: Velocity,
    boidAcc :: Acceleration
    } deriving (Show)

data World = World {
    config      :: Config,
    timestamp   :: Float, 
    boids       :: [Boid]
    }

data Config = Config 
    { cfgScreenWidth         :: Int
    , cfgScreenHeight        :: Int
    , cfgFPS                 :: Int

    , cfgNumBoids            :: Int
    , cfgMaxSpeed            :: Float
    , cfgMaxForce            :: Float
    , cfgNeighborRadius      :: Float
    , cfgBoundaryMargin      :: Float

    , cfgWSeparation         :: Float
    , cfgWAlignment          :: Float
    , cfgWCohesion           :: Float
    , cfgWBoundary           :: Float
    }


defaultConfig :: Config
defaultConfig = Config
    { cfgScreenWidth         = 800
    , cfgScreenHeight        = 600
    , cfgFPS                 = 60

    , cfgNumBoids            = 70
    , cfgMaxSpeed            = 150.0
    , cfgMaxForce            = 10.0
    , cfgNeighborRadius      = 100.0
    , cfgBoundaryMargin      = 100.0

    , cfgWSeparation         = 1.8
    , cfgWAlignment          = 2
    , cfgWCohesion           = 1.8
    , cfgWBoundary           = 3
}

screeWidth, screeHeight :: Int
screeWidth = cfgScreenWidth defaultConfig
screeHeight = cfgScreenHeight defaultConfig


window :: Display
window = InWindow "Boid Algorithms!!" (screeWidth, screeHeight) (100,100)

background :: Color
background = black

fps :: Int
fps = cfgFPS defaultConfig

maxSpeed :: Float
maxSpeed = cfgMaxSpeed defaultConfig

maxForce :: Float
maxForce = cfgMaxForce defaultConfig

neighborRadius :: Float
neighborRadius = cfgNeighborRadius defaultConfig

main :: IO ()
main = simulate window background fps initialWorld drawWorld updateWorld


initialWorld :: World
initialWorld = World defaultConfig 0.0 initialBoids
    where
        numBoids = cfgNumBoids defaultConfig
        initialGen = mkStdGen 42 -- Seed
        (posGen, velGen) = splitGen initialGen

        (xPosGen, yPosGen) = splitGen posGen
        (vxPosGen, vyPosGen) = splitGen velGen

        halfW = fromIntegral screeWidth / 2
        halfH = fromIntegral screeHeight / 2

        randXs = randomRs (-halfW * 0.8, halfW * 0.8) xPosGen
        randYs = randomRs (-halfH * 0.8, halfH * 0.8) yPosGen

        randVXs = randomRs (-maxSpeed, maxSpeed) vxPosGen
        randVYs = randomRs (-maxSpeed, maxSpeed) vyPosGen

        positions = zipWith (\x y -> Position (V2 x y)) randXs randYs
        velocaties = zipWith (\vx vy -> Velocity (V2 vx vy)) randVXs randVYs

        initialBoids = zipWith3 createBoid [1..numBoids] positions velocaties


drawWorld :: World -> Picture
drawWorld = Pictures . map drawSingleBoid . boids 


updateWorld :: ViewPort -> Float -> World -> World
updateWorld _ dt oldWorld = 
    let 
        cfg = config oldWorld
        currentTime = timestamp oldWorld
        oldBoids = boids oldWorld
        newBoids = map (updateSingleBoid cfg dt oldBoids) oldBoids
    in oldWorld { timestamp = currentTime + dt, boids = newBoids}


createBoid :: Int -> Position -> Velocity -> Boid
createBoid id' pos vel = Boid {
    boidId = id',
    boidPos = pos,
    boidVel = vel,
    boidAcc = Acceleration (V2 0 0)
}

unitBoidPicture :: Picture
unitBoidPicture = Polygon [(12, 0), (-5, -6), (-5, 6)]


drawSingleBoid :: Boid -> Picture
drawSingleBoid boid =
    let
        Position (V2 px py) = boidPos boid
        Velocity (V2 vx vy) = boidVel boid
        Acceleration (V2 ax ay) = boidAcc boid
        angle = atan2 vy vx * (180 / pi)

        boidModel = GL.Rotate (-angle) $ GL.Color white unitBoidPicture
        velocityLine = GL.Color red $ GL.Line [(0, 0), (vx * 0.5, vy * 0.5)]

        -- debugText =
        --     -- 讓文字小一點，才不會擠在一起
        --     GL.Scale 0.1 0.1 $
        --     -- 把文字往右邊平移一點，避免跟 Boid 重疊
        --     GL.Translate 150 0 $
        --     -- 藍色文字
        --     GL.Color blue $
        --     -- 顯示內容： "v=(vx,vy), ang=angle"
        --     Text ( "p=(" ++ show (round px) ++ "," ++ show (round py) ++ ")" ++ 
        --             ",v=(" ++ show (round vx) ++ "," ++ show (round vy) ++ ")," ++
        --             "a=(" ++ show (round ax) ++ "," ++ show (round ay) ++ ")"
        --         )

    in GL.Translate px py  $ GL.Pictures [boidModel, velocityLine] --debugText]

updateSingleBoid :: Config -> Float -> [Boid] -> Boid -> Boid
updateSingleBoid cfg dt allboids boid = 
    let 

        Position oldPos = boidPos boid
        Velocity oldVec = boidVel boid

        Acceleration acc = calculateForces cfg boid allboids

        updatedVel =  oldVec + (acc ^* dt)
        limiatedVel = limit (cfgMaxSpeed cfg) updatedVel

        newPos = oldPos + (limiatedVel ^* dt)

    in boid {
        boidPos = Position newPos,
        boidVel = Velocity limiatedVel,
        boidAcc = Acceleration acc
        }


calculateForces :: Config -> Boid -> [Boid] -> Acceleration
calculateForces cfg boid allboids =
    let 
        neighbors = findBoidNeighbors cfg boid allboids
        
        wSeparation = cfgWSeparation cfg
        wAlignment = cfgWAlignment cfg
        wCohesion = cfgWCohesion cfg
        wBoundary = cfgWBoundary cfg

        forceCohesion       = ruleCohesion cfg boid neighbors
        forceSeparation     = ruleSeparation cfg boid neighbors
        forceAlignment      = ruleAlignment cfg boid neighbors
        forceBoundary       = ruleBoundaryAvoidance cfg boid

        accer = (forceCohesion ^* wCohesion) + 
                (forceSeparation ^* wSeparation) + 
                (forceAlignment   ^* wAlignment) + 
                (forceBoundary    ^* wBoundary)
    in Acceleration accer


findBoidNeighbors :: Config -> Boid -> [Boid] -> [Boid]
findBoidNeighbors cfg mainBoid = filter isPotentialNeighbor
    where
        isPotentialNeighbor otherBoid = 
            (boidId mainBoid /= boidId otherBoid) && 
            (let 
                Position mainPos = boidPos mainBoid
                Position otherPos = boidPos otherBoid
             in 
                quadrance (mainPos - otherPos) <= cfgNeighborRadius cfg * cfgNeighborRadius cfg
            )

steer :: Config -> Boid -> V2 Float -> V2 Float
steer cfg boid desired = 
    let Velocity currentVel = boidVel boid
        steeringForce = desired - currentVel
    in limit (cfgMaxForce cfg) steeringForce

ruleSeparation :: Config -> Boid -> [Boid] -> V2 Float
ruleSeparation _ _ [] = V2 0 0
ruleSeparation cfg boid neighbors = 
    let 
        Position myPos  = boidPos boid
        sumOfRepulsions = foldl' addRepulsionForce  (V2 0.0 0.0) neighbors 
            where
                addRepulsionForce acc other =
                    let 
                        Position otherPos = boidPos other
                        diff =  myPos - otherPos
                        dist = norm diff
                    in if dist > 0 
                        then acc + (normalize diff ^/ dist)
                        else acc
        -- avgRepulsion = if null neighbors 
        --                 then V2 0 0 
        --                 else sumOfRepulsions ^/ fromIntegral (length neighbors)
          
    in 
        if quadrance sumOfRepulsions > 0
        then 
            let
                desired = normalize sumOfRepulsions ^* cfgMaxSpeed cfg
                in steer cfg boid desired
        else V2 0 0
        

ruleCohesion :: Config -> Boid -> [Boid] -> V2 Float
ruleCohesion _ _ [] = V2 0.0 0.0
ruleCohesion cfg boid neighbors = 
    let
        Position myPos = boidPos boid
        totalCohesionForce = foldl' addCohesionForce (V2 0.0 0.0) neighbors
            where
                addCohesionForce acc other =
                    let Position otherPos = boidPos other
                    in acc + otherPos
        centerOfMass = totalCohesionForce ^/ fromIntegral (length neighbors)
        desiredVec = centerOfMass - myPos
    in
        if quadrance desiredVec > 0 
        then
            let desired = normalize desiredVec ^* cfgMaxSpeed cfg
            in steer cfg boid desired
        else V2 0 0
         

ruleAlignment :: Config -> Boid -> [Boid] -> V2 Float
ruleAlignment _ _ [] = V2 0.0 0.0
ruleAlignment cfg boid neighbors = 
    let 
        avgVelocity  = foldl' calAliNeighborForce (V2 0.0 0.0) neighbors ^/ fromIntegral (length neighbors)
            where
                calAliNeighborForce acc other =
                    let Velocity otherVel = boidVel other
                    in acc + otherVel
    in 
        if quadrance avgVelocity > 0
        then 
            let desired = normalize avgVelocity ^* cfgMaxSpeed cfg
            in steer cfg boid desired
        else V2 0 0

ruleBoundaryAvoidance :: Config -> Boid -> V2 Float
ruleBoundaryAvoidance cfg boid =
    let 
        Position (V2 px py) = boidPos boid
        margin = cfgBoundaryMargin cfg

        halfWidth = fromIntegral screeWidth / 2 
        halfHeight = fromIntegral screeHeight / 2 

        fx      | px < -halfWidth + margin  =  maxSpeed
                | px > halfWidth - margin   =  -maxSpeed
                | otherwise                 = 0
        fy      | py < -halfHeight + margin =  maxSpeed
                | py > halfHeight - margin  =  -maxSpeed
                | otherwise                 = 0
        desiredVel2 = V2 fx fy
    in
        if quadrance desiredVel2 > 0
        then steer cfg boid desiredVel2
        else V2 0 0

limit :: Float -> V2 Float -> V2 Float
limit maxLen vec = 
    if quadrance vec > maxLen * maxLen
    then normalize vec ^* maxLen
    else vec
