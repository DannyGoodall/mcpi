#www.stuffaboutcode.com
#github.com/martinohanlon/minecraft-stuff
#Raspberry Pi, Minecraft - Minecraft 'stuff' extensions

from . import minecraft
from . import block
import copy
import os
import time
import collections
import math
import sqlite3


class MinecraftDrawing:
    #: MinecraftDrawing class.  Useful functions for drawing objects.
    def __init__(self, mc):
        self.mc = mc

        #: Expose pep8 naming compliant methods
        self.draw_point_3d = self.drawPoint3d
        self.draw_face = self.drawFace
        self.draw_vertices = self.drawVertices
        self.draw_line = self.drawLine
        self.draw_sphere = self.drawSphere
        self.draw_circle = self.drawCircle
        self.draw_horizontal_circle = self.drawHorizontalCircle

    def find_point_on_sphere(self, cx, cy, cz, horizontal_angle, vertical_angle, radius):
        # Moved from Minecraft Turtle as it has value as a generic routine
        x = cx + (radius * (math.cos(math.radians(vertical_angle)) * math.cos(math.radians(horizontal_angle))))
        y = cy + (radius * (math.sin(math.radians(vertical_angle))))
        z = cz + (radius * (math.cos(math.radians(vertical_angle)) * math.sin(math.radians(horizontal_angle))))
        return x, y, z

    def drawPoint3d(self, x, y, z, block_type, block_data=0):
        # draw point
        self.mc.setBlock(x, y ,z ,block_type ,block_data)

    def drawFace(self, vertices, filled, block_type, block_data=0):
        # draws a face, when passed a collection of vertices which make up a polyhedron
        def keyX(point):
            return point.x

        def keyY(point):
            return point.y

        def keyZ(point):
            return point.z

        # get the edges of the face
        edges_vertices = []
        # persist first vertex
        first_vertex = vertices[0]
        # get last vertex
        last_vertex = vertices[0]
        # loop through vertices and get edges
        for vertex in vertices[1:]:
            # got 2 vertices, get the points for the edge
            edges_vertices = edges_vertices + self.getLine(
                last_vertex.x,
                last_vertex.y,
                last_vertex.z,
                vertex.x,
                vertex.y,
                vertex.z
            )
            # persist the last vertex found    
            last_vertex = vertex
        # get edge between the last and first vertices
        edges_vertices = edges_vertices + self.getLine(
            last_vertex.x,
            last_vertex.y,
            last_vertex.z,
            first_vertex.x,
            first_vertex.y,
            first_vertex.z
        )

        if filled:
            #draw solid face
            # sort edges vertices
            edges_vertices.sort(key=keyZ)
            edges_vertices.sort(key=keyY)
            edges_vertices.sort(key=keyX)

            #draw lines between the points on the edges
            # this algorithm isnt very efficient, but it does always fill the gap
            last_vertex = edges_vertices[0]
            for vertex in edges_vertices[1:]:
                # got 2 vertices, draw lines between them
                self.drawLine(
                    last_vertex.x,
                    last_vertex.y,
                    last_vertex.z,
                    vertex.x,
                    vertex.y,
                    vertex.z,
                    block_type,
                    block_data
                )
                # persist the last vertex found
                last_vertex = vertex

        else:
            # draw wireframe
            self.drawVertices(edges_vertices, block_type, block_data)
        
    def drawVertices(self, vertices, block_type, block_data=0):
        # draw's all the points in a collection of vertices with a block
        for vertex in vertices:
            self.drawPoint3d(vertex.x, vertex.y, vertex.z, block_type, block_data)

    def drawLine(self, x1, y1, z1, x2, y2, z2, block_type, block_data=0):
        # draw line
        self.drawVertices(
            self.getLine(x1, y1, z1, x2, y2, z2), block_type, block_data
        )

    def drawSphere(self, x1, y1, z1, radius, block_type, block_data=0):
        # draw sphere
        # create sphere
        for x in range(radius*-1, radius):
            for y in range(radius*-1, radius):
                for z in range(radius*-1, radius):
                    if x**2 + y**2 + z**2 < radius**2:
                        self.drawPoint3d(x1 + x, y1 + y, z1 + z, block_type, block_data)

    def drawCircle(self, x0, y0, z, radius, block_type, block_data=0):
        # draw a verticle circle
        f = 1 - radius
        ddf_x = 1
        ddf_y = -2 * radius
        x = 0
        y = radius
        self.drawPoint3d(x0, y0 + radius, z, block_type, block_data)
        self.drawPoint3d(x0, y0 - radius, z, block_type, block_data)
        self.drawPoint3d(x0 + radius, y0, z, block_type, block_data)
        self.drawPoint3d(x0 - radius, y0, z, block_type, block_data)
     
        while x < y:
            if f >= 0:
                y -= 1
                ddf_y += 2
                f += ddf_y
            x += 1
            ddf_x += 2
            f += ddf_x   
            self.drawPoint3d(x0 + x, y0 + y, z, block_type, block_data)
            self.drawPoint3d(x0 - x, y0 + y, z, block_type, block_data)
            self.drawPoint3d(x0 + x, y0 - y, z, block_type, block_data)
            self.drawPoint3d(x0 - x, y0 - y, z, block_type, block_data)
            self.drawPoint3d(x0 + y, y0 + x, z, block_type, block_data)
            self.drawPoint3d(x0 - y, y0 + x, z, block_type, block_data)
            self.drawPoint3d(x0 + y, y0 - x, z, block_type, block_data)
            self.drawPoint3d(x0 - y, y0 - x, z, block_type, block_data)

    def drawHorizontalCircle(self, x0, y, z0, radius, block_type, block_data=0):
        # draw a horizontal circle
        f = 1 - radius
        ddf_x = 1
        ddf_z = -2 * radius
        x = 0
        z = radius
        self.drawPoint3d(x0, y, z0 + radius, block_type, block_data)
        self.drawPoint3d(x0, y, z0 - radius, block_type, block_data)
        self.drawPoint3d(x0 + radius, y, z0, block_type, block_data)
        self.drawPoint3d(x0 - radius, y, z0, block_type, block_data)
     
        while x < z:
            if f >= 0:
                z -= 1
                ddf_z += 2
                f += ddf_z
            x += 1
            ddf_x += 2
            f += ddf_x   
            self.drawPoint3d(x0 + x, y, z0 + z, block_type, block_data)
            self.drawPoint3d(x0 - x, y, z0 + z, block_type, block_data)
            self.drawPoint3d(x0 + x, y, z0 - z, block_type, block_data)
            self.drawPoint3d(x0 - x, y, z0 - z, block_type, block_data)
            self.drawPoint3d(x0 + z, y, z0 + x, block_type, block_data)
            self.drawPoint3d(x0 - z, y, z0 + x, block_type, block_data)
            self.drawPoint3d(x0 + z, y, z0 - x, block_type, block_data)
            self.drawPoint3d(x0 - z, y, z0 - x, block_type, block_data)

    def getLine(self, x1, y1, z1, x2, y2, z2):
        # returns points on a line
        # 3d implementation of bresenham line algorithm

        # return step
        def sign(a):
            if a < 0:
                return -1
            elif a > 0:
                return 1
            elif a == 0:
                return 0

        # list for vertices
        vertices = []

        # if the 2 points are the same, return single vertice
        if x1 == x2 and y1 == y2 and z1 == z2:
            vertices.append(minecraft.Vec3(x1, y1, z1))
        else:
            # else get all points in edge
            dx = x2 - x1
            dy = y2 - y1
            dz = z2 - z1

            ax = abs(dx) << 1
            ay = abs(dy) << 1
            az = abs(dz) << 1

            sx = sign(dx)
            sy = sign(dy)
            sz = sign(dz)

            x = x1
            y = y1
            z = z1

            # x dominant
            if ax >= max(ay, az):
                yd = ay - (ax >> 1)
                zd = az - (ax >> 1)
                loop = True
                while loop:
                    vertices.append(minecraft.Vec3(x, y, z))
                    if x == x2:
                        loop = False
                    if yd >= 0:
                        y += sy
                        yd -= ax
                    if zd >= 0:
                        z += sz
                        zd -= ax
                    x += sx
                    yd += ay
                    zd += az
            # y dominant
            elif ay >= max(ax, az):
                xd = ax - (ay >> 1)
                zd = az - (ay >> 1)
                loop = True
                while loop:
                    vertices.append(minecraft.Vec3(x, y, z))
                    if y == y2:
                        loop=False
                    if xd >= 0:
                        x += sx
                        xd -= ay
                    if zd >= 0:
                        z += sz
                        zd -= ay
                    y += sy
                    xd += ax
                    zd += az
            # z dominant
            elif az >= max(ax, ay):
                xd = ax - (az >> 1)
                yd = ay - (az >> 1)
                loop = True
                while loop:
                    vertices.append(minecraft.Vec3(x, y, z))
                    if z == z2:
                        loop=False
                    if xd >= 0:
                        x += sx
                        xd -= az
                    if yd >= 0:
                        y += sy
                        yd -= az
                    z += sz
                    xd += ax
                    yd += ay
                    
        return vertices


class MinecraftShape:
    # MinecraftShape - a class for managing shapes

    def __init__(self, mc, position, shapeBlocks, visible=True):
        #persist data
        self.mc = mc
        #shape blocks is the original shape
        self.shapeBlocks = shapeBlocks
        #drawn shape blocks is where the blocks have been drawn
        self.drawnShapeBlocks = None
        #set it to visible or not
        self.visible = visible
        #store the position
        self.position = position
        #move the shape to its position
        self.move(position.x, position.y, position.z)

    def draw(self):
        #draw the shape

        #Find the blocks which are different between the last ones drawn
        #create counters
        drawnCounter = collections.Counter(self.drawnShapeBlocks)
        currentCounter = collections.Counter(self.shapeBlocks)
        
        #work out the blocks which need to be cleared
        for blockToClear in drawnCounter - currentCounter:
            #print "block to clear"
            #print str(blockToClear.actualPos.x) + "," + str(blockToClear.actualPos.y) + "," + str(blockToClear.actualPos.z)
            self.mc.setBlock(blockToClear.actualPos.x, blockToClear.actualPos.y, blockToClear.actualPos.z, block.AIR.id)

        #work out the blocks which have changed and need to be re-drawn
        for blockToDraw in currentCounter - drawnCounter:
            #print "block to draw"
            #print str(blockToDraw.actualPos.x) + "," + str(blockToDraw.actualPos.y) + "," + str(blockToDraw.actualPos.z)
            self.mc.setBlock(blockToDraw.actualPos.x, blockToDraw.actualPos.y, blockToDraw.actualPos.z, blockToDraw.blockType, blockToDraw.blockData)

        #OLD CODE, USED PRIOR TO THE CODE ABOVE WHICH ONLY CHANGES THE BLOCKS WHICH HAVE CHANGED    
        #clear all blocks
        #self.clear()
        
        #work out which blocks to draw
        #if self.drawnShapeBlocks == None:
        #    blocksToDraw = copy.deepcopy(self.shapeBlocks)

        #for blockToDraw in blocksToDraw:
        #    self.mc.setBlock(blockToDraw.actualPos.x,
        #                     blockToDraw.actualPos.y,
        #                     blockToDraw.actualPos.z,
        #                     blockToDraw.blockType,
        #                     blockToDraw.blockData)
        
        #update the blocks which have been drawn
        self.drawnShapeBlocks = copy.deepcopy(self.shapeBlocks)
        self.visible = True

    def clear(self):
        #clear the shape
        if self.drawnShapeBlocks == None:
            pass
        else:
            for blockToClear in self.drawnShapeBlocks:
                self.mc.setBlock(blockToClear.actualPos.x,
                                 blockToClear.actualPos.y,
                                 blockToClear.actualPos.z,
                                 block.AIR.id)
            self.drawnShapeBlocks = None
        self.visible = False

    def moveBy(self, x, y, z):
        #move the position of the shape by x,y,z
        self.move(self.position.x + x, self.position.y + y, self.position.z + z)

    def move(self, x, y, z):
        #move the position of the shape to x,y,z
        self.position.x = x
        self.position.y = y
        self.position.z = z

        #recalulate the shapeBlockPositions based on the shapeBlocks and the position
        #loop through the shapeBlocks
        for shapeBlock in self.shapeBlocks:
            #offset the position of the block by the position
            shapeBlock.actualPos.x = shapeBlock.relativePos.x + self.position.x
            shapeBlock.actualPos.y = shapeBlock.relativePos.y + self.position.y
            shapeBlock.actualPos.z = shapeBlock.relativePos.z + self.position.z
        
        #if its visible redraw it
        if self.visible:
            self.draw()

# a class created to manage a block within a shape
class ShapeBlock():
    def __init__(self, x, y, z, blockType, blockData=0):
        #persist data
        self.blockType = blockType
        self.blockData = blockData
        #store the positions
        # relative pos - block position relatively to other shape blocks
        self.relativePos = minecraft.Vec3(x, y, z)
        # actual pos - actual block position in the world
        self.actualPos = minecraft.Vec3(x, y, z)
        # the mc block object
        self.mcBlock = block.Block(blockType, blockData)

    def __hash__(self):
        return hash((self.actualPos.x, self.actualPos.y, self.actualPos.z, self.blockType, self.blockData))

    def __eq__(self, other):
        return (
                   self.actualPos.x, self.actualPos.y, self.actualPos.z, self.blockType, self.blockData
               ) == (
            other.actualPos.x, other.actualPos.y, other.actualPos.z, other.blockType, other.blockData
        )

class MinecraftTurtleDrawing(MinecraftDrawing):
    def __init__(self, mc, session_reference=None, persist=False):
        super(MinecraftTurtleDrawing, self).__init__(mc)
        self.cache_progress = True
        # additional_block_vector describes the delta from the block being drawn. If x,y and z are 0 - i.e. there
        # is no delta, then nothing is drawn. If additional_block_vector describes another position in space,
        # then a block described by additional_block is drawn in the minecraft world at the current position +
        # the delta described by additional_block_vector
        self.additional_block = block.TORCH
        self.additional_block_vector = minecraft.Vec3(0, 0, 0)
        self.active_cache = None
        self.cache_paused = False
        self.block_cache = {}
        self.initialise_cache()
        self.db_file = "mccache.db"
        self._db = None
        self._cursor = None
        self.persist = persist
        if self.persist and session_reference is None:
            raise ValueError("Cannot persist session to DB because a session reference has not been supplied.")
        elif self.persist:
            self.initialise_db()
            self.db_session_id = self.db_create_new_session(session_reference)["id"]
            self.set_active_cache(session_reference)

    def initialise_db(self):
        create_schema = not os.path.exists(self.db_file)
        self._db = sqlite3.connect(self.db_file)
        self._db.row_factory = sqlite3.Row
        self._cursor = self._db.cursor()
        # Allow access to columns by dictionary addressing

        if create_schema:
            with self._db:
                self._db.execute(
                    """
                    CREATE TABLE session (
                      id                    INTEGER PRIMARY KEY AUTOINCREMENT,
                      reference             TEXT,
                      created               DATETIME DEFAULT CURRENT_TIMESTAMP,
                      UNIQUE (id)
                    )
                    """
                )
                self._db.execute(
                    """
                    CREATE TABLE vector (
                      session_id            INTEGER,
                      id                    INTEGER PRIMARY KEY AUTOINCREMENT,
                      x                     INTEGER,
                      y                     INTEGER,
                      z                     INTEGER,
                      block_id              INTEGER,
                      block_data            INTEGER,
                      was_block_id          INTEGER,
                      was_block_data        INTEGER,
                      UNIQUE (session_id, id)
                    )
                    """
                )

    def db_replay_session_iter(self, session_id):
        self._cursor.execute("SELECT * FROM vector WHERE session_id=? ORDER BY id DESC", (session_id,))
        rows = self._cursor.fetchall()
        for row in rows:
            yield row

    def db_replay_was_blocks(self, session_id):
        current_state = self.cache_progress
        self.cache_progress = False
        for row in self.db_replay_session_iter(session_id):
            self.drawPoint3d(row["x"], row["y"], row["z"], row["was_block_id"], row["was_block_data"])
        self.cache_progress = current_state

    def db_replay_blocks(self, session_id):
        current_state = self.cache_progress
        self.cache_progress = False
        for row in self.db_replay_session_iter(session_id):
            self.drawPoint3d(row["x"], row["y"], row["z"], row["block_id"], row["block_data"])
        self.cache_progress = current_state

    def db_return_last_row(self):
        last_id = self._cursor.lastrowid
        self._cursor.execute("SELECT * FROM session WHERE rowid=?", (last_id,))
        row = self._cursor.fetchone()

        return row

    def db_create_new_session(self, reference):
        with self._db:
            r = self._cursor.execute("INSERT INTO session (reference) values (?)", (reference,))
        return self.db_return_last_row()

    def db_create_new_vector(self, session_id, x, y, z, block_id, block_data, was_block_id, was_block_data):
        with self._db:
            r = self._cursor.execute("""
                INSERT INTO vector (session_id, x, y, z, block_id, block_data, was_block_id, was_block_data)
                  values (?, ?, ?, ?, ?, ?, ?, ?)
              """,
              (session_id, x, y, z, block_id, block_data, was_block_id, was_block_data)
            )

        return self.db_return_last_row()

    def db_store_session(self, reference):
        pass

    def initialise_cache(self, cache_name=None):
        if cache_name is None:
            cache_name = "default"
        if cache_name not in self.block_cache:
            self.block_cache[cache_name] = []

    def set_active_cache(self, cache_name=None):
        self.active_cache = cache_name
        self.initialise_cache(self.active_cache)

    def get_cache_iter(self, cache_name=None):
        if cache_name is None:
            cache_name="default"
        if cache_name not in self.block_cache:
            raise ValueError("Cache "+cache_name+" not found.")
        for vector_and_block in self.block_cache[cache_name]:
            yield vector_and_block[0], vector_and_block[1]

    def set_blocks_from_cache(self, cache_name=None, block_id=block.AIR.id, block_data=0):
        if cache_name is None:
            cache_name="default"

        # Remember what state the cache was in
        current_pause_status = self.cache_paused
        # Pause the cache so that the replay from the cache does continue to record block builds
        self.cache_paused = True
        for vector_and_block in self.get_cache_iter(cache_name):
            vector = vector_and_block[0]
            block_details = vector_and_block[1]
            self.draw_point_3d(
                vector.x,
                vector.y,
                vector.z,
                block_id,
                block_data
            )
        # Restore the cache
        self.cache_paused = current_pause_status

    def record_cache_block(self, x, y, z, block_id, block_data, was_block_id, was_block_data):
        self.block_cache[self.active_cache].append(
            (
                minecraft.Vec3(x, y, z),
                block.Block(block_id,  block_data )
            )
        )
        if self.persist:
            self.db_create_new_vector(self.db_session_id, x, y, z, block_id, block_data, was_block_id, was_block_data)

    def drawPoint3d(self, x, y, z, block_type, block_data=0):
        current_block = self.mc.getBlockWithData(x, y, z)
        if self.cache_progress:
            self.pre_draw_point_3d(x, y, z, block_type, block_data, current_block.data, current_block.id)

        self.mc.setBlock(x, y, z, block_type, block_data)
        if self.cache_progress:
            self.post_draw_point_3d(x, y, z, block_type, block_data, current_block.data, current_block.id)

    def pre_draw_point_3d(self, x, y, z, block_type, block_data, was_block_id, was_block_data):
        pass

    def post_draw_point_3d(self, x, y, z, block_type, block_data, was_block_id, was_block_data):
        if self.active_cache and not self.cache_paused:
            self.record_cache_block(x, y, z, block_type, block_data, was_block_data, was_block_id)


class MinecraftTurtle:

    SPEEDTIMES = {
        0:  0,
        11: 0.05,
        12: 0.025,
        13: 0.0125,
        14: 0.006,
        15: 0.0003,
        10: 0.1,
        9:  0.2,
        8:  0.3,
        7:  0.4,
        6:  0.5,
        5:  0.6,
        4:  0.7,
        3:  0.8,
        2:  0.9,
        1:  1
    }

    def __init__(self, mc, position = minecraft.Vec3(0,0,0), session_reference=None, persist=False):
        # Set defaults
        self.mc = mc

        # The reference for this session so that we can track the blocks built
        self.session_reference = session_reference
        self.persist = persist

        # Start position
        self.startposition = position

        # Set turtle position
        self.position = position

        # Set turtle angles
        self.heading = 0
        self.verticalheading = 0

        # Set pen down
        self._pendown = True

        # Set pen block to black wool
        self._penblock = block.Block(block.WOOL.id, 15)

        # Flying to true
        self.flying = True

        # Set speed
        self.turtlespeed = 6

        # Create turtle
        self.showturtle = True

        # Create drawing object
        self.mcDrawing = MinecraftTurtleDrawing(self.mc, self.session_reference, self.persist)

        # Set turtle block
        self.turtleblock = block.Block(block.DIAMOND_BLOCK.id)

        # Draw turtle
        self._drawTurtle(
            int(self.position.x),
            int(self.position.y),
            int(self.position.y)
        )

    def set_additional_block(self, block_description):
        # Set the additional block that will be drawn relative to the current turtle position
        self.mcDrawing.additional_block = block_description

    def set_additional_block_vector(self, vector):
        # Set the delta vector for the additional block to be drawn relative to the turtle position
        self.mcDrawing.additional_block_vector = vector

    def countdown(self, count_from, message=None, delay=1):
        if message:
            self.mc.postToChat(message)
        for x in range(count_from, 0, -1):
            self.mc.postToChat(str(x))
            time.sleep(delay)

    def forward(self, distance):
        # Get end of line
        x,y,z = self.mcDrawing.find_point_on_sphere(
            self.position.x,
            self.position.y,
            self.position.z,
            self.heading,
            self.verticalheading,
            distance
        )
        # Move turtle forward
        self._moveTurtle(x,y,z)

    def backward(self, distance):
        # Move turtle backward
        # Get end of line
        x,y,z = self.mcDrawing.find_point_on_sphere(
            self.position.x,
            self.position.y,
            self.position.z,
            self.heading,
            self.verticalheading - 180,
            distance
        )
        # Move turtle forward
        self._moveTurtle(x,y,z)

    def _moveTurtle(self,x,y,z):
        # Get blocks between current position and next
        targetX, targetY, targetZ = int(x), int(y), int(z)
        # If walking, set target Y to be height of world
        if not self.flying:
            targetY = self.mc.getHeight(targetX, targetZ)
        currentX, currentY, currentZ = int(self.position.x), int(self.position.y), int(self.position.z)

        # Clear the turtle
        if self.showturtle:
            self._clearTurtle(currentX, currentY, currentZ)

        # If speed is 0 and flying, just draw the line, else animate it
        if self.turtlespeed == 0 and self.flying:
            # Draw the line
            if self._pendown:
                self.mcDrawing.drawLine(
                    currentX,
                    currentY - 1,
                    currentZ,
                    targetX,
                    targetY - 1,
                    targetZ,
                    self._penblock.id,
                    self._penblock.data
                )
        else:
            # Otherwise we need to animate the drawing of the line
            blocksBetween = self.mcDrawing.getLine(currentX, currentY, currentZ, targetX, targetY, targetZ)
            for blockBetween in blocksBetween:
                # If walking update the y, to be the height of the world
                if self.flying == False:
                    blockBetween.y = self.mc.getHeight(blockBetween.x, blockBetween.z)
                # Draw the turtle
                if self.showturtle:
                    self._drawTurtle(
                        blockBetween.x,
                        blockBetween.y,
                        blockBetween.z
                    )
                # Draw the pen
                if self._pendown:
                    self.mcDrawing.drawPoint3d(
                        blockBetween.x,
                        blockBetween.y - 1,
                        blockBetween.z,
                        self._penblock.id,
                        self._penblock.data
                    )
                # Wait
                time.sleep(self.SPEEDTIMES[self.turtlespeed])
                # Clear the turtle
                if self.showturtle:
                    self._clearTurtle(blockBetween.x, blockBetween.y, blockBetween.z)

        # Update turtle's position to be the target
        self.position.x, self.position.y, self.position.z = x,y,z
        # Draw turtle
        if self.showturtle:
            self._drawTurtle(
                targetX,
                targetY,
                targetZ
            )

    def right(self, angle):
        # Rotate turtle angle to the right
        self.heading = self.heading + angle
        if self.heading > 360:
            self.heading -= 360

    def right_interior(self, angle):
        # Turn right and create an interior angle of angle degress
        self.right(180-angle)

    def left(self, angle):
        # Rotate turtle angle to the left
        self.heading = self.heading - angle
        if self.heading < 0:
            self.heading += 360

    def left_interior(self, angle):
        # Turn left and create an interior angle of angle degress
        self.left(180-angle)

    def circle(self, radius):
        self.mcDrawing.drawHorizontalCircle(
            self.position.x,
            self.position.y-1,
            self.position.z,
            radius,
            self._penblock.id,
            self._penblock.data
        )

    def polygon(self, sides, length, right=True, forward=True):
        # Draw a polygon of sides 'sides' with a side length of 'length', will default to drawing forwards
        # and turning right, but can be changed using right and forward logical parameters
        for n in range(sides):
            if forward:
                self.forward(length)
            else:
                self.backward(length)
            if right:
                self.right_interior(self.interior_angle(sides))
            else:
                self.left_interior(self.interior_angle(sides))

    def interior_angle(self, sides):
        # Calculate the interior angle of a polygon with 'sides' sides
        return (sides-2) * 180 / sides

    def up(self, angle):
        # Rotate turtle angle up
        self.verticalheading += angle
        if self.verticalheading > 360:
            self.verticalheading -= 360
        # Turn flying on
        if not self.flying:
            self.flying = True

    def down(self, angle):
        # Rotate turtle angle down
        self.verticalheading -=angle
        if self.verticalheading < 0:
            self.verticalheading += 360
        # Turn flying on
        if not self.flying:
            self.flying = True

    def ascend(self, amount=1):
        self.position.y += amount

    def descend(self, amount=1):
        self.position.y -= amount

    vertically_up = ascend
    vertically_down = descend

    def height_every(self, multiple, override_height=None):
        inty = int(self.position.y)
        return inty % multiple == 0 \
            if override_height is None else int(override_height) % multiple == 0

    def setx(self, x):
        self.setposition(x, self.position.y, self.position.z)

    def sety(self, y):
        self.setposition(self.position.x, y, self.position.z)

    def setz(self, z):
        self.setposition(self.position.x, self.position.y, z)

    def setposition(self, x, y, z):
        # Clear the turtle
        if self.showturtle:
            self._clearTurtle(self.position.x, self.position.y, self.position.z)
        # Update the position
        self.position.x = x
        self.position.y = y
        self.position.z = z
        # Draw the turtle
        if self.showturtle:
            self._drawTurtle(
                self.position.x,
                self.position.y,
                self.position.z
            )

    def setheading(self, angle):
        self.heading = angle

    def setverticalheading(self, angle):
        self.verticalheading = angle
        # Turn flying on
        if self.flying == False:
            self.flying = True

    def home(self):
        self.position.x = self.startposition.x
        self.position.y = self.startposition.y
        self.position.z = self.startposition.z

    def pendown(self):
        self._pendown = True

    def penup(self):
        self._pendown = False

    def isdown(self):
        return self.pendown

    def fly(self):
        self.flying = True

    def walk(self):
        self.flying = False
        self.verticalheading = 0

    def penblock(self, block_id, block_data = 0):
        self._penblock = block.Block(block_id, block_data)

    def use_block(self, block_name):


    def speed(self, turtlespeed):
        self.turtlespeed = turtlespeed

    def _drawTurtle(self,x,y,z):
        # Draw turtle

        # Get the current state of the cache progress flag
        current_state = self.mcDrawing.cache_progress
        # Turn off the caching
        self.mcDrawing.cache_progress = False
        # Draw the turtle
        self.mcDrawing.drawPoint3d(
            x,
            y,
            z,
            self.turtleblock.id,
            self.turtleblock.data
        )
        # Restore the state of the cache
        self.mcDrawing.cache_progress=current_state

    def _clearTurtle(self,x,y,z):
        # Clear turtle

        # Get the current state of the cache progress flag
        current_state = self.mcDrawing.cache_progress
        # Turn off the caching
        self.mcDrawing.cache_progress = False
        # Clear the turtle
        self.mcDrawing.drawPoint3d(x, y, z, block.AIR.id)
        self.mcDrawing.cache_progress=current_state

    def _roundXYZ(x,y,z):
        return int(round(x,0)), int(round(y,0)), int(round(z,0))

    def _roundVec3(position):
        return minecraft.Vec3(int(position.x), int(position.y), int(position.z))


# testing
if __name__ == "__main__":

    #connect to minecraft
    mc = minecraft.Minecraft.create()

    #clear area
    mc.setBlocks(-25, 0, -25, 25, 25, 25, block.AIR.id)

    #create drawing object
    mcDrawing = MinecraftDrawing(mc)

    #line
    mcDrawing.drawLine(0,0,-10,-10,10,-5,block.STONE.id)

    #circle
    mcDrawing.drawCircle(-15,15,-15,10,block.WOOD.id)

    #sphere
    mcDrawing.drawSphere(-15,15,-15,5,block.OBSIDIAN.id)
    
    #face - solid triangle
    faceVertices = []
    faceVertices.append(minecraft.Vec3(0,0,0))
    faceVertices.append(minecraft.Vec3(5,10,0))
    faceVertices.append(minecraft.Vec3(10,0,0))
    mcDrawing.drawFace(faceVertices, True, block.SNOW_BLOCK.id)

    #face - wireframe square
    faceVertices = []
    faceVertices.append(minecraft.Vec3(0,0,5))
    faceVertices.append(minecraft.Vec3(10,0,5))
    faceVertices.append(minecraft.Vec3(10,10,5))
    faceVertices.append(minecraft.Vec3(0,10,5))
    mcDrawing.drawFace(faceVertices, False, block.DIAMOND_BLOCK.id)

    #face - 5 sided shape
    faceVertices = []
    faceVertices.append(minecraft.Vec3(0,15,0))
    faceVertices.append(minecraft.Vec3(5,15,5))
    faceVertices.append(minecraft.Vec3(3,15,10))
    faceVertices.append(minecraft.Vec3(-3,15,10))
    faceVertices.append(minecraft.Vec3(-5,15,5))
    mcDrawing.drawFace(faceVertices, True, block.GOLD_BLOCK.id)

    #test shape
    playerPos = mc.player.getTilePos()

    #create shape object
    shapeBlocks = [ShapeBlock(0,0,0,block.DIAMOND_BLOCK.id),
                  ShapeBlock(1,0,0,block.DIAMOND_BLOCK.id),
                  ShapeBlock(1,0,1,block.DIAMOND_BLOCK.id),
                  ShapeBlock(0,0,1,block.DIAMOND_BLOCK.id),
                  ShapeBlock(0,1,0,block.DIAMOND_BLOCK.id),
                  ShapeBlock(1,1,0,block.DIAMOND_BLOCK.id),
                  ShapeBlock(1,1,1,block.DIAMOND_BLOCK.id),
                  ShapeBlock(0,1,1,block.DIAMOND_BLOCK.id)]
    # move the shape about
    myShape = MinecraftShape(mc, playerPos, shapeBlocks)
    time.sleep(10)
    myShape.moveBy(-1,1,-1)
    time.sleep(10)
    myShape.moveBy(1,0,1)
    time.sleep(10)
    myShape.moveBy(1,1,0)
    time.sleep(10)

    #clear the shape
    myShape.clear()
