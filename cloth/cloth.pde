// JS version for embedding in html


/**
* Cloth simulator with Position-Based Dynamics.
* Based on [Muller06]
*
* Uses processing v3.3.5
*
* new in version 0.2
* gravity/external force
* locked vertices
* mesh
*
* new in version 0.3
* collision detection 
* friction
*
* new in version 0.4
* small fixes
*
*
* @version 0.4
* @author Nils Lenart
*/

// correct corners no twisting DONE

PFont f;
Cloth cloth;
SphereObj sphere;

int width = 20;
int length = 20;
float spacing = 15;
int z = 0;


//SETUP FUNCTIONS
void setup(){

	size(1000, 700, OPENGL);

	//Initialize font
    f = createFont("Arial",16,true);

	//Initialize sphere
	sphere = new SphereObj(new Vector(500,500,0), 100, this);

	//Initialize cloth
	cloth = new Cloth(this, sphere, width, length, spacing);
}



//MAIN LOOP
void draw() {
	background(255);
	lights();
	writeText();


	//for staging a screenshot
	/*
	if (cloth.stageMode) {
	cloth.setPosition(0,0, new Vector(400, 500, 50));
	cloth.setPosition(width-1, 0, new Vector(600, 500, 50));
	cloth.setPosition(0,length-1, new Vector(400, 500, -50));
	cloth.setPosition(width-1,length-1, new Vector(600, 500, -50));
	}
	*/

	//sets two corners of cloth to mouse position
	cloth.setPosition(0,0,new Vector(mouseX, mouseY, z));
	cloth.setPosition(0, width-1, new Vector(mouseX+(width-1)*spacing, mouseY, z));

	// //updates the state of the cloth and draws
	cloth.update();
	cloth.draw();

	// //draws the sphere
	// sphere.draw();
}



//Writes text to screen
void writeText() {
    textFont(f,15);
    fill(0);
    text("Cloth simulator v0.4",20,30,0);
	text("Locked vertex (s): "+cloth.staticMode.toString(), 20, 50, 0);
	text("Gravity (g): "+cloth.gravityMode.toString(), 20, 70, 0);
	text("Mesh (m): "+cloth.meshMode.toString(), 20, 90, 0);
	text("Solver iterations (i): "+cloth.iterNum.toString(), 20, 110, 0);
	text("Reset (r)", 20, 130, 0);
}


//CHECK FOR KEY-PRESS
void keyPressed() {
    if (key == 's') {cloth.toggleStatic(); cloth.staticMode = !cloth.staticMode;}
    if (key == 'g') {cloth.gravityMode = !cloth.gravityMode;}
    if (key == 'm') {cloth.meshMode = !cloth.meshMode;}
	if (key == 'i') {cloth.iterNum = (cloth.iterNum%10)+1;}
	if (key == 'z') {z = (z%10)+1;}
	if (key == 'p') {cloth.vertices[0][0].position.print();}
	if (key == 'r') {cloth = new Cloth(this, sphere, width, length, spacing);}
	if (key == 'x') {
	    cloth.stageMode = !cloth.stageMode;
	    cloth.vertices[0][length-1].w = (cloth.vertices[0][length-1].w+1)%2;
	    cloth.vertices[width-1][length-1].w = (cloth.vertices[width-1][length-1].w+1)%2;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/* 
*	Copied from Cloth.java
*/

/* The cloth class creates a cloth object which consists of vertices interconnected by distance constraints.
 * The vertices can be interpolated with triangles to draw a mesh
 *
 */
class Cloth {
    PApplet p;
    int width;
    int length;
    Vertex[][] vertices;
    ArrayList<DistanceConstraint> constraints;
    ArrayList<CollisionConstraint> collisionConstraints;
    SphereObj sphere;

    float timestep = (float)0.1; //Timestep size
    Vector g;

    Boolean staticMode = true;   //Sets the two corner vertices to "infinite" mass
    Boolean gravityMode = true;  //Gravity added before solver
    Boolean meshMode = true;     //Draw the cloth as mesh
    Boolean stageMode = false;   //easter egg...
    Integer iterNum = 3;         //Number of solver iterations
    
    Cloth (PApplet parent, SphereObj sphere, int width, int length, float spacing) {
		p = parent;
		this.width = width;
		this.length = length;
		Vector corner = new Vector(0,0,0);
		constraints = new ArrayList<DistanceConstraint>();
		this.sphere = sphere;
		
		//Gravity acceleration
		g = new Vector(0,(float)1,0);

		//Initialize size of arrays
		vertices = new Vertex[width][length];

		//Populate vertices and constraints
		for (int i = 0; i < width; i++) {
		    for (int j = 0; j < length; j++) {
			
			vertices[i][j] = new Vertex (corner.copy(), 1);
			corner.add(new Vector(spacing,0,0));
			

			//Add constraint along width
			if (i > 0) {
			    constraints.add(new DistanceConstraint(vertices[i-1][j], vertices[i][j], spacing, true));
			}
			//add constraint along length
			if (j > 0) {
			    constraints.add(new DistanceConstraint(vertices[i][j-1], vertices[i][j], spacing, true));
			}

		    }
		    corner.x = 0;
		    corner.add(new Vector(0,spacing,0));
		}

		//LOCK CORNERS
		vertices[0][0].w = 0;
		vertices[0][width - 1].w = 0;
		
    }
    

    //Update state
    void update() {	
		if (gravityMode) {gravity();}
		solve(iterNum);
    }

    //Draw cloth
    void draw() {
		if (meshMode) {drawMesh();}
		else {drawVertices();}
    }


    //Add gravity as external force.
    public void gravity () {
		for (int i = 0; i < width; i++) {
		    for (int j = 0; j < length; j++) {
			vertices[i][j].velocity.add(g);
		    }
		}
    }

    //From Muller 2006
    void solve(int iterNum) {
		//apply external forces to all vertices
		

		//damp all velocities

		//get vector of positions p <- x + deltaT*v
		for (int i = 0; i < width; i++) {
		    for (int j = 0; j < length; j++) {
				vertices[i][j].setP(timestep);
		    }
		}

		//Generate collision constraints
		// collisionConstraints = new ArrayList<CollisionConstraint>();
		// for (int i = 0; i < width; i++) {
		//     for (int j = 0; j < length; j++) {
		// 		if(sphere.collides(vertices[i][j])) {
		// 		    collisionConstraints.add(new CollisionConstraint(sphere.position.copy(),vertices[i][j],sphere.radius+5, false));
		// 		}
		//     }
		// }

		//Modify p to satisfy all constrains, iterate n times
		for (int iter = 0; iter < iterNum; iter++) {
		    for (int i = 0; i < constraints.size(); i++) {
				constraints.get(i).solve();
		    }
		  //   for (int i = 0; i < collisionConstraints.size(); i++) {
				// collisionConstraints.get(i).solve();
		  //   }
		}
		
		//Update position and velocities of all vertices
		for (int i = 0; i < width; i++) {
		    for (int j = 0; j < length; j++) {
				vertices[i][j].update(timestep);
		    }
		}
		
		
		//Add friction to colliding vertices
		// Take 10% off colliding vertices
		// for (int i = 0; i < collisionConstraints.size(); i++) {
		//     collisionConstraints.get(i).getVertex().velocity.scale((float)0.7);
	 //    }
		
    }



    //Draw vertices as ellipses
    void drawVertices () {
       	for (int i = 0; i < width; i++) {
	    for (int j = 0; j < length; j++) {
		p.pushMatrix();		
		float x = vertices[i][j].position.x;
		float y = vertices[i][j].position.y;
		float z = vertices[i][j].position.z;
		p.translate(x,y,z);
		p.ellipse(0,0,1,1); //0,0 as i,j? instead of translate
		p.popMatrix();
	    }
	}
    }

    //Draw vertices interpolated as triangle mesh
    void drawMesh () {
	//draw shape
	p.beginShape(TRIANGLE_STRIP);
	p.fill(128);
	for (int i = 1; i < width; i++) {
	    
	    //WEAVING
	    if (i%2 == 0) {
		for (int j = 0; j < length; j++) {
		    p.vertex(vertices[i-1][j].position.x,vertices[i-1][j].position.y,vertices[i-1][j].position.z);
		    p.vertex(vertices[i][j].position.x,vertices[i][j].position.y,vertices[i][j].position.z);
		}
	    }
	    else {
		for (int j = length-1; j >= 0; j--) {
		    p.vertex(vertices[i-1][j].position.x,vertices[i-1][j].position.y,vertices[i-1][j].position.z);
		    p.vertex(vertices[i][j].position.x,vertices[i][j].position.y,vertices[i][j].position.z);
		}
	    }
	
	}
	p.endShape();
    }

    //TOGGLES CORNER VERTEX STATIC
    void toggleStatic () {
		// vertices[0][0].w = (vertices[0][0].w+1)%2;
		// vertices[width-1][0].w = (vertices[width-1][0].w+1)%2;
		vertices[0][0].w = Number.MAX_VALUE;		
		vertices[0][width-1].w = Number.MAX_VALUE;
    }

    //Position setter of corners
    void setPosition(int index1, int index2, Vector pos) {
		vertices[index1][index2].position = pos.copy();
    }

}



//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


/*
*	Copied from CollisionConstraint.java
*/
class CollisionConstraint {
    private Vector p;
    private Vertex b;
    private float d;
    private boolean equality;

    CollisionConstraint (Vector p, Vertex b, float d, boolean equality)
    {
	this.p = p;
	this.b = b;
	this.d = d;
	this.equality = equality;
    }

    boolean isEquality () {
	return equality;
    }

    float evaluate () {
	Vector BA = p.copy();
	BA.subtract(b.p);
	return BA.getMagnitude()-d;
    }

    
    void solve () {
	//SKIP SATISFIED INEQUALITIES
	if (!isEquality() && (evaluate() > 0)) {
	    return;
	}

	float reciprocal = 1/(b.w);
	float constraint = evaluate();
	Vector BANorm = p.copy();
	BANorm.subtract(b.p);
	BANorm.normalize();
	
	
	Vector deltaBp = BANorm.copy();
	deltaBp.scale(b.w*reciprocal*constraint);
	
	
	b.p.add(deltaBp);

    }

    Vertex getVertex() {return b;}
    
}



//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*
*	Copied from DistanceConstraint.java
*/
class DistanceConstraint {
    private Vertex a;
    private Vertex b;
    private float d;
    private boolean equality;

    DistanceConstraint (Vertex a, Vertex b, float d, boolean equality) {
	this.a = a;
	this.b = b;
	this.d = d;
	this.equality = equality;
    }

    boolean isEquality () {
	return equality;
    }

    float evaluate () {
	Vector BA = a.p.copy();
	BA.subtract(b.p);
	return BA.getMagnitude()-d;
    }

    
    void solve () {
	//SKIP SATISFIED INEQUALITIES
	if (!isEquality() && (evaluate() > 0)) {
	    return;
	}

	float reciprocal = 1/(a.w+b.w);
	float constraint = evaluate();
	Vector BANorm = a.p.copy();
	BANorm.subtract(b.p);
	BANorm.normalize();
	
	Vector deltaAp = BANorm.copy();
	deltaAp.scale(-a.w*reciprocal*constraint);
	
	Vector deltaBp = BANorm.copy();
	deltaBp.scale(b.w*reciprocal*constraint);
	
	a.p.add(deltaAp);
	b.p.add(deltaBp);

    }
    
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

/*
*	Copied from SphereObj.java
*/	
class SphereObj {
    Vector position;
    float radius;
    PApplet p;
    SphereObj (Vector position, float radius, PApplet parent) {
	this.radius = radius;
	p = parent;
	this.position = position;
    }


    void draw() {
	p.pushMatrix();
	p.translate(position.x,position.y,position.z);
	p.sphere(radius);
	p.popMatrix();
    }

    boolean collides (Vertex v) {
		Vector testVector = position.copy();
		testVector.subtract(v.position);
		if (testVector.getMagnitude() <= radius) {
			return true;
		} else {
			return false;
		}
		// return true;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


/*
*	Copied from Vector.java
*/

/**
 *  Vector class
 *  @version 2.0
 */
class Vector {
   float x;
   float y;
   float z;

   //Empty constructor
   Vector() {
      x = 0;
      y = 0;
      z = 0;
   }

   //General constructor
   Vector(float x, float y, float z) {
      this.x = x;
      this.y = y;
      this.z = z;
   }

   //////
   //////

   //Returns a copy of this
   Vector copy () {
      return new Vector(x,y,z);
   }


   void add (Vector vec) {
      x += vec.x;
      y += vec.y;
      z += vec.z;
   }


   void subtract (Vector vec) {
      x -= vec.x;
      y -= vec.y;
      z -= vec.z;
   }

   void scale (float factor) {
    x *= factor;
    y *= factor;
    z *= factor;
  }

   //Returns the magnitude of a vector
   float getMagnitude () {
      // return PApplet.sqrt(PApplet.sq(x)+PApplet.sq(y)+PApplet.sq(z));
      return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
   }


   //Scales a vector to a given magnitude
   void setMagnitude (float mag) {

      float factor = mag/getMagnitude();
      scale(factor);
   }

    
    //Normalizes a vector
    void normalize (){
	setMagnitude((float)1.);
    }


    void print () {
	System.out.println("x: "+x+". y: "+y+". z: "+z+".");
    }

}
   //////
   ////// (in outermost layer)

   //STATIC FUNCTIONS
   //Computes the dot product of two vectors
   float dotProduct (Vector vec1, Vector vec2) {
      return (vec1.x*vec2.x+vec1.y*vec2.y+vec1.z*vec2.z);
   }

   //Computes the cross product of two vectors.
   Vector crossProduct (Vector u, Vector v) {
      float newX = u.y*v.z - u.z*v.y;
      float newY = u.z*v.x - u.x*v.z;
      float newZ = u.x*v.y - u.y*v.x;
      return new Vector(newX,newY,newZ);
   }

    //Returns the distance between two vectors
    float getDistance(Vector v, Vector w) {
        Vector distance = v.copy();
        distance.subtract(w);
        float distancef = distance.getMagnitude();
        return distancef;
    }
//}



//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////


/*
*	Copied from Vertex.java
*/


class Vertex {

    float w;
    Vector position;
    Vector velocity;
    Vector p;

    Vertex(Vector initialPosition, float mass) {
	position = initialPosition;
	velocity = new Vector(0,0,0);
	w = 1/mass;

    }
    
    //GENERATE P USED FOR SOLVING CONSTRAINTS
    void setP(float timestep) {
	p = position.copy();
	Vector scaledVelocity = velocity.copy();
	scaledVelocity.scale(timestep);
	p.add(scaledVelocity);
    }

    //UPDATE POSITION AND VELOCITY AFTER SOLVING THE CONSTRAINTS
    void update(float timestep) {
	Vector deltaPos = p.copy();
	deltaPos.subtract(position);
	deltaPos.scale(1/timestep);

	velocity = deltaPos.copy();
	position = p.copy();
    }
    
}
