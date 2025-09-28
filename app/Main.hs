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
    timestamp :: Float, 
    boids :: [Boid]
    } deriving (Show)

screeWidth, screeHeight :: Int
screeWidth = 1200
screeHeight = 900


window :: Display
window = InWindow "Boid Algorithms!!" (screeWidth, screeHeight) (100,100)

background :: Color
background = black

fps :: Int
fps = 60

main :: IO ()
main = simulate window background fps initialWorld drawWorld updateWorld


initialWorld :: World
initialWorld = World 0.0 initialBoids
    where
        numBoids = 70
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
        currentTime = timestamp oldWorld
        oldBoids = boids oldWorld
        newBoids = map (updateSingleBoid dt oldBoids) oldBoids
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


maxSpeed :: Float
maxSpeed = 150.0

maxForce :: Float
maxForce = 5.0

neighborRadius :: Float
neighborRadius = 75.0

updateSingleBoid :: Float -> [Boid] -> Boid -> Boid
updateSingleBoid dt allboids boid = 
    let 
        Position oldPos = boidPos boid
        Velocity oldVec = boidVel boid

        Acceleration acc = calculateForces boid allboids

        updatedVel =  oldVec + (acc ^* dt)
        limiatedVel = limit maxSpeed updatedVel

        newPos = oldPos + (limiatedVel ^* dt)

    in boid {
        boidPos = Position newPos,
        boidVel = Velocity limiatedVel,
        boidAcc = Acceleration acc
        }


calculateForces :: Boid -> [Boid] -> Acceleration
calculateForces boid allboids =
    let 
        neighbors = findBoidNeighbors boid allboids
        
        wSeparation = 5.0
        wAlignment = 4
        wCohesion = 3.0
        wBoundary = 5

        forceCohesion       = ruleCohesion boid neighbors
        forceSeparation     = ruleSeparation boid neighbors
        forceAlignment      = ruleAlignment boid neighbors
        forceBoundary       = ruleBoundaryAvoidance boid

        accer = (forceCohesion ^* wCohesion) + 
                (forceSeparation ^* wSeparation) + 
                (forceAlignment   ^* wAlignment) + 
                (forceBoundary    ^* wBoundary)
    in Acceleration accer


findBoidNeighbors :: Boid -> [Boid] -> [Boid]
findBoidNeighbors mainBoid = filter isPotentialNeighbor
    where
        isPotentialNeighbor otherBoid = 
            (boidId mainBoid /= boidId otherBoid) && 
            (let 
                Position mainPos = boidPos mainBoid
                Position otherPos = boidPos otherBoid
             in 
                quadrance (mainPos - otherPos) <= neighborRadius * neighborRadius
            )

steer :: Boid -> V2 Float -> V2 Float
steer boid desired = 
    let Velocity currentVel = boidVel boid
        steeringForce = desired - currentVel
    in limit maxForce steeringForce

ruleSeparation :: Boid -> [Boid] -> V2 Float
ruleSeparation _ [] = V2 0 0
ruleSeparation boid neighbors = 
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
        avgRepulsion = if null neighbors 
                        then V2 0 0 
                        else sumOfRepulsions ^/ fromIntegral (length neighbors)
          
    in 
        if quadrance avgRepulsion > 0
        then 
            let
                desired = normalize avgRepulsion ^* maxSpeed
                in steer boid desired
        else V2 0 0
        

ruleCohesion :: Boid -> [Boid] -> V2 Float
ruleCohesion _ [] = V2 0.0 0.0
ruleCohesion boid neighbors = 
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
            let desired = normalize desiredVec ^* maxSpeed
            in steer boid desired
        else V2 0 0
         

ruleAlignment :: Boid -> [Boid] -> V2 Float
ruleAlignment _ [] = V2 0.0 0.0
ruleAlignment boid neighbors = 
    let 
        avgVelocity  = foldl' calAliNeighborForce (V2 0.0 0.0) neighbors ^/ fromIntegral (length neighbors)
            where
                calAliNeighborForce acc other =
                    let Velocity otherVel = boidVel other
                    in acc + otherVel
    in 
        if quadrance avgVelocity > 0
        then 
            let desired = normalize avgVelocity ^* maxSpeed
            in steer boid desired
        else V2 0 0



ruleBoundaryAvoidance :: Boid -> V2 Float
ruleBoundaryAvoidance boid =
    let 
        Position (V2 px py) = boidPos boid
        margin = 100.0 
        turnForce = 20.0

        halfWidth = fromIntegral screeWidth / 2 
        halfHeight = fromIntegral screeHeight / 2 
        
        fx  | px < -halfWidth + margin  = turnForce
            | px > halfWidth - margin   = -turnForce
            | otherwise                 = 0

        fy  | py < -halfHeight + margin = turnForce
            | py > halfHeight - margin  = -turnForce
            | otherwise                 = 0
    in
        V2 fx fy

limit :: Float -> V2 Float -> V2 Float
limit maxLen vec = 
    if quadrance vec > maxLen * maxLen
    then normalize vec ^* maxLen
    else vec
