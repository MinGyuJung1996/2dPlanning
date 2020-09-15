#pragma once

/*

MS2D's overall structure:
	Computes minkowski sum between "model 1 rotated by n degree" & "model 2"
		model numbers are inside : global pair<int,int> ModelInfo_CurrentModel
		degree is stored inside  : global int			ModelInfo_CurrentFrame
	Key 3 functions
		void initialize():
			reads from file & save data to global. (also save segmented / approximated version)
			need to be called once per program run.
		void postProcess(int FirstModel, int SecondModel):
			rotate 1st model(+disk) [0, 360) degrees and save it.
			need to be called every time when model 1 or 2 is changed. (since change in model 2 might result in flipping model 1)
			Note that model 1 might be flipped, depending on the indices(2 params) passed.
		void minkowskisum(int frame, int figure2):
			compute minkowski sum ... (same as above)
			alot are from global variable.
	How to Use:
		Assuming that the model itself doesn't keep deforming... 
			just do init & postProcess once.
			than call minkowski sum. multiple times for every rotation.
			use 2 resulting global var?
	Initial input seems to be B-splines,
	Final Output seems to be sets of arcs (each set forms a loop i guess.)
		The output is therefore constant curvature
		And resides in only one quadrant (given O is circle center)

MAT's overall structure
	Key functions
		void Lee::init()	(= init_mat() call in main.cpp)
			load files.
			call once in program.
		void Lee::draw()	(= draw_mat() call in main.cpp)
			segment curves and call mat.

TODO
	change this solution to include source codes from MS2D project or vice versa.
	DONE: outer boundary? - clue in draw's object 5.
	what about boundary? can we compute minkowski with boundary (of the whole scene)?
	find upper/lower bound in maximal_disk_search, for circular arc.

	Look at the ***TOPOLOGY*** of MS2D output & voronoi input
		Voronoi seems to be well ordered...
		MS2D seems to be locally aligned but not globally...
			Model_Result[i][j].Arcs[k] represents a single arc
			order of k-values seems to be well aligned (Test of comparing x components succeeded all(8*8*360) for fabs()<1e-4 in APPENDIX 1)
				arc[k]'s endpoint == arc[k+1]'s beginning point
			order of j-values aren't consisten
				
			order of i-values doesn't matter (since different loop)
		MS2D models_approx seems to be continuous			(search with : //debug : test if models_APPROX is continuous)
		MS2D models_rotated_approx seems to be continuous	(search with : //debug : test if models_rotated_APPROX is continuous)
			but if (flip with y-axis is applied in postprocess...) it might not be -> but it seems okay to assume that inputs of MS are continuous...
			--> so there is somewhere in MS that ruins the topology...
		ANS : The main problem was in making convolutionArc
			if the radius of RHS was bigger than that of LHS in the "reverse case", x[0] & x[1] should be swapt to preserve CCW/CW
			A minor Prob was : ordering of ArcSplines in a loop (not arcs of an arcspline) -> this maybe CW or CCW by random. -> need to find a better way, currently 3/23000 prob to fail.

	About the CircularArc data structure...
		Normals : points from circle center to boundary		(search with : CircularArc::CircularArc(Point & i, Point & e, Point & t) )
			concavity, convexity of arc will not change normal direction
		Bez -> Biarc approximation :						(search std::pair<CircularArc, CircularArc> BezierCrv::BiArc() )
		x[0], x[1] order :
			follows that of traversing the model boundary ccw (for model_approx)
				a[0].x[0] -> a[0].x[1] -> a[1].x[0] -> ... following this order == traversing the model ccw
			= follows that of original bezier curve ( = x[0] will be B(0) at some subdived curved B) 

	About the ArcSpline data structure...
		Quadrant : set by normals							(search with : ArcSpline::ArcSpline(BezierCrv & Crv) )
			concavity, convexity will not change quadrant.
		CCW : bool ccw = outer(cp[1]-cp[0], cp[3]-cp[2]) >= 0
		Arcs in AS follows the order of original bez curve... (model_approx)
			Arcs[0] -> Arcs[1] -> Arcs[2] ... traversing (in a arcspline) = traversing boundary ccw (model_approx) 
	
	Q. from arcSpline's quadrant & ccw ... can we decide to flip it?
		No. from quad/ccw : we can decide inside is left/right... but we don't know which should be inside, just with that arc
		Rather See building conv arc part and find where in/out is not preserved 
			
	DONE: Voronoi seems to distinguish which region (in/out of curve loop) to use with cw/ccw... but which is which???
		-> if curve is traversed ccw -> use inside of curve to draw Voronoi cells.
		-> if curve is traversed cw  -> use outside
		Note that input files in both Voronoi & MS2d are originally CCW

	DONE: MS2D with multiple models?
		MS2D has some limits to possible positions (of models), due to grid?
			trnslating the model by x+=0.1 has no problem, but with x+=1.0 crashes... in the grid
		-> resolving the problem with grid, it works... see image in _notes folder.

	GRID SIZE?
	Trimming test
		not only used in ms but also at convolution

	Integrity test?

	MS2D's CW/CCW stuff...
		In ArcSplines's constructor, bool ccw = outer(cp[1]-cp[0], cp[3]-cp[2]) >= 0;
		Arcsplines in approx models are constructed with monotone-bezier-segments... so arcs in a arcSpline share same ccw (Tested with Appendix2)
		a link maybe used later? https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
		original data are ccw (models_approx)

	deque<ArcSpline> has a same element in first & last? -> not always... i guess

	findMaximalDiskSharingPoint
		check newly invented  findMaximalDiskSharingPoint works okay?
		what if disk grows too big that  it touches other part of the arc?
		mode stuff not imple
		2-7-2 may be reduced... since it is not gonna be the final output anyway.

	if(p00.first.distance(p11.first)<1e-9){
	p=osculating(bezier[0].geometry(0)).p;
	swap(p,q);
		can't really understand why...

	a dedicated version of findMaximal for recFindBisector

	ERROR CHECKS for MS OUT?
		- is r > 0

BUG
	1. CW circularArc's normal are swapt.
		see : CircularArc::CircularArc(Point & i, Point & e, Point & t)
	2. Wrong normals are referenced, for CW circularArc, in : void Convolution_ArcSpline(std::vector<ArcSpline> &source, ArcSpline & lhs, ArcSpline & rhs, int ls[2], int rs[2], bool reverse)
		actually bug 1 and 2 cancels out each other... (maybe 1 was made for 2)

OPTIMIZATION?
	ordering MS2D output?
	delete debug elements in  CircularArc
	Error checking part of [MS2D out->Voronoi In]
	done : memory free part commentated. (8 of minkowski sum())
	*** a list of arcs is given, we don't need to compute maximal disks for all arcs, just when monotone curvature breaks... but curvature graph is discrete...(kind of like bunch of step functions...)
	<minor>  r2 in findMaximalDiskSharingPoint, /2 in loop... may be done after finishing.
	instead of testing every arc, cluster arcs with monotone curvature. + GRID?
	subdivCircular arc function calls can be embedded





// APPENDIX

1. test1 : result above
for (size_t i = 0; i < Model_Result.size(); i++)
	{
		for (size_t j = 0;  j < Model_Result[i].size();  j++)
		{
			for (size_t k = 0; k < Model_Result[i][j].Arcs.size() - 1; k++)
			{
				float t = Model_Result[i][j].Arcs[k].x[1].P[0] - Model_Result[i][j].Arcs[k + 1].x[0].P[0];
				if (fabs(t) > 1e-4) std::cout << "k not continuous ...? " << t << " " << i << " " << j << " " << k << std::endl;
			}
		}
	}


2. test2 : result : no error message: arcs in a arcSpline share same ccw
for (auto i : in)
		{
			bool flip = i.Arcs[0].ccw; //flip if ccw // assumes Arcs share same ccw

			auto temp = i.Arcs;
			for (auto j : i.Arcs)
			{
				dbg if (j.ccw != flip) cout << "ERROR : Arcspline not as assumed" << endl;
			}
		}


3. CURRENT TOPOLOGY CONSOLE OUTPUT
fake func
minkowski_id
minkowski_id
minkowski_id
minkowski_id
minkowski_id
minkowski_id
minkowski_id
ERR : convertMsOutput_Clockwise's count = 0 at7, 2, 26
minkowski_id
ERROR : sth is not cont. @ 2285 with model info 7 7 1
x values at i = 2285   0.954926, 0.955126 with x,y diff 0.000191633, -0.000206306
ERR : convertMsOutput_Clockwise's count = 0 at7, 7, 312

--> 7 7 1 is not prob of ordering.






*/