/*
 * Cloth Simulation using a relaxed constraints solver
 */

// Suggested Readings

// Advanced Character Physics by Thomas Jakobsen Character
// http://freespace.virgin.net/hugo.elias/models/m_cloth.htm
// http://en.wikipedia.org/wiki/Cloth_modeling
// http://cg.alexandra.dk/tag/spring-mass-system/
// Real-time Cloth Animation http://www.darwin3d.com/gamedev/articles/col0599.pdf

var DAMPING = 0.03;
var DRAG = 1 - DAMPING;
var MASS = 0.1;
var restDistance = 25;

//var xSegs = 20;
//var ySegs = 20;

var GRAVITY = 981 * 1.4;
var gravity = new THREE.Vector3( 0, - GRAVITY, 0 ).multiplyScalar( MASS );

var TIMESTEP = 18 / 1000;
var TIMESTEP_SQ = TIMESTEP * TIMESTEP;


//var ballPosition = new THREE.Vector3( 0, - 45, 0 );
//var ballSize = 60; //40



function Cloth(width, height ) {
	this.geometry;
	this.ball;
	this.width = width;
	this.height = height;
	this.pins = [];
	this.tmpForce = new THREE.Vector3();
	this.lastTime;
	this.diff = new THREE.Vector3();

	this.collided_before = [];
	this.first_contact_vel = new THREE.Vector3(0,0,0);
	this.first_contact_point = [];

	this.wind = true;
	this.windStrength = 100;
	this.windForce = new THREE.Vector3( 0, 0, 0 );


	w = segs;// || 10;
	h = segs;// || 10;
	this.w = w;
	this.h = h;

	this.particles = [];
	this.constraints = [];
	this.u, this.v;
	var clothFunction = plane( restDistance * segs, restDistance * segs );

	this.setGeometry = function(geometry) {
		this.geometry = geometry;
	}

	this.setBall = function(ball) {
		this.ball = ball;
	}
	// Create particles
	for ( v = 0; v <= this.h; v ++ ) {
		for ( u = 0; u <= this.w; u ++ ) {
			this.particles.push(
				new Particle( u / this.w, v / this.h, 0, MASS )
			);
			this.collided_before.push(false);
		}
	}

	// Structural
	for ( v = 0; v < this.h; v ++ ) {
		for ( u = 0; u < this.w; u ++ ) {
			this.constraints.push( [
				this.particles[ index( u, v ) ],
				this.particles[ index( u, v + 1 ) ],
				restDistance
			] );
			this.constraints.push( [
				this.particles[ index( u, v ) ],
				this.particles[ index( u + 1, v ) ],
				restDistance
			] );
		}
	}
	for ( u = this.w, v = 0; v < this.h; v ++ ) {
		this.constraints.push( [
			this.particles[ index( u, v ) ],
			this.particles[ index( u, v + 1 ) ],
			restDistance
		] );
	}
	for ( v = this.h, u = 0; u < this.w; u ++ ) {
		this.constraints.push( [
			this.particles[ index( u, v ) ],
			this.particles[ index( u + 1, v ) ],
			restDistance
		] );
	}


	function index( u, v ) {
		return u + v * ( this.w + 1 );
	};


	function plane() {
		return function(u,v) {
			var x = ( u - 0.5 ) * width;
			var y = ( v + 0.5 ) * height;
			var z = 0;
			//console.log(x + " " +y);

			return new THREE.Vector3( x, y, z );
		}
	};

	function Particle( x, y, z, mass ) {
		//console.log(this.clothFunction(3,4));
		this.position = clothFunction( x, y ); // position
		//console.log(x + " " + y);
		this.previous = clothFunction( x, y ); // previous
		this.original = clothFunction( x, y );
		this.a = new THREE.Vector3( 0, 0, 0 ); // acceleration
		this.mass = mass;
		this.invMass = 1 / mass;
		this.tmp = new THREE.Vector3();
		this.tmp2 = new THREE.Vector3();
	};

	// Force -> Acceleration
	Particle.prototype.addForce = function( force ) {
		this.a.add(
			this.tmp2.copy( force ).multiplyScalar( this.invMass )
		);
	};

	// Performs Verlet integration
	Particle.prototype.integrate = function( timesq ) {
		var newPos = this.tmp.subVectors( this.position, this.previous );
		newPos.multiplyScalar( DRAG ).add( this.position );
		newPos.add( this.a.multiplyScalar( timesq ) );
		this.tmp = this.previous;
		this.previous = this.position;
		this.position = newPos;
		this.a.set( 0, 0, 0 );
	};


	this.satisifyConstraints = function( p1, p2, distance ) {
		this.diff.subVectors( p2.position, p1.position );
		var currentDist = this.diff.length();
		if ( currentDist === 0 ) return; // prevents division by 0
		var correction = this.diff.multiplyScalar( 1 - distance / currentDist );
		var correctionHalf = correction.multiplyScalar( 0.5 );
		p1.position.add( correctionHalf );
		p2.position.sub( correctionHalf );
	}

	this.simulate = function( time ) {
		if ( ! this.lastTime ) {
			this.lastTime = time;
			return;
		}

		var i, il, particles, particle, pt, constraints, constraint;
		// Aerodynamics forces
		if ( this.wind ) {
			var face, faces = this.geometry.faces, normal;
			particles = this.particles;
			for ( i = 0, il = faces.length; i < il; i ++ ) {
				face = faces[ i ];
				normal = face.normal;
				this.tmpForce.copy( normal ).normalize().multiplyScalar( normal.dot( this.windForce ) );
				particles[ face.a ].addForce( this.tmpForce );
				particles[ face.b ].addForce( this.tmpForce );
				particles[ face.c ].addForce( this.tmpForce );
			}
		}

		for ( particles = this.particles, i = 0, il = particles.length; i < il; i ++ ) {
			particle = particles[ i ];
			particle.addForce( gravity );
			particle.integrate( TIMESTEP_SQ );
		}

		// Start Constraints
		constraints = this.constraints;
		il = constraints.length;
		for ( i = 0; i < il; i ++ ) {
			constraint = constraints[ i ];
			this.satisifyConstraints( constraint[ 0 ], constraint[ 1 ], constraint[ 2 ] );
		}

		// Ball Constraints
		//ballPosition.z = - Math.sin( Date.now() / 600 ) * 90 ; //+ 40;
		//ballPosition.x = Math.cos( Date.now() / 400 ) * 70;
		//if ( sphere.visible ) {
		for ( particles = this.particles, i = 0, il = particles.length; i < il; i ++ ) {
			//console.log(ball.position);
			particle = particles[ i ];
			var pos = particle.position;
			//console.log(pos);
			this.diff.subVectors( pos, this.ball.position );
			//console.log(this.diff);
			//console.log("diff: " + this.diff.z);
			if ( this.diff.length() < ballSize || this.collided_before[i]) {
				// collided

				if (this.first_contact_point != undefined && this.ball.position.z < this.first_contact_point.z){
					this.diff.normalize().multiplyScalar( -1 * ballSize );
				} else {
					this.diff.normalize().multiplyScalar( ballSize );
				}
				pos.copy(this.ball.position).add(this.diff);
				//console.log("2: ");
				//console.log(this.diff);
				//console.log(this.diff.length());
				if (!this.collided_before[i]) {
					this.first_contact_vel = this.ball.getLinearVelocity();
					this.first_contact_point[i] = new THREE.Vector3(pos.x, pos.y, pos.z);
				}
				this.collided_before[i] = true;

				//var velc = (new THREE.Vector3).copy(this.ball.getLinearVelocity());
				//this.ball.getLinearVelocity().sub(new THREE.Vector3(1000, 1000, 1000));

				//var diffOrigin = new THREE.Vector3().subVectors(particle.original, this.ball.position);
				//if (diffOrigin.length() > 5 * ballSize) {
				//ball.applyCentralImpulse(this.ball.getLinearVelocity().normalize().multiplyScalar(-5));
				if (this.ball.getLinearVelocity().z < 200) {
					var sum = 0;
					for (var c=0; c<this.collided_before.length; c++) {
						if (this.collided_before[i])
							++sum;
					}
					ball.applyCentralImpulse(new THREE.Vector3(0, 0, 60000/sum));
				}
				//console.log(this.ball.getLinearVelocity().length());
				//if (this.ball.getLinearVelocity().z > 0 && this.ball.getLinearVelocity().length() > this.first_contact_vel.divideScalar(-2).length()) {
//					this.collided_before = false;
//			  }
				//var diffFirst = new THREE.Vector3().subVectors(pos, this.first_contact_point[i]);
				var diffFirst = Math.abs(pos.z - this.first_contact_point[i].z); //new THREE.Vector3().subVectors(pos, this.first_contact_point[i]);
//				console.log(this.ball.position);
				console.log(pos);
				console.log(this.first_contact_point[i]);
				console.log(diffFirst);
				if (game.state == GameState.READY || (this.ball.getLinearVelocity().z > 0 && diffFirst < 100)) {
					this.collided_before[i] = false;
				}
			}
		}
		//}

		// Pin Constraints
		for ( i = 0, il = this.pins.length; i < il; i ++ ) {
			var xy = this.pins[ i ];
			var p = this.particles[ xy ];
			//console.log(p);
			p.position.copy( p.original );
			p.previous.copy( p.original );
		}
	}

	//this.particles = particles;
	//this.constraints = constraints;
//
	this.clothFunction = clothFunction;
//
	//this.index = index;
//
	//this.plane = plane;
//
	//this.Particle = Particle;

	//this.satisifyConstraints = satisifyConstraints;

	//this.simulate = simulate;
}
