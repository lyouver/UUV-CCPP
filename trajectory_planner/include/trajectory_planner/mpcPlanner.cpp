/*
	FILE: mpcPlanner.cpp
	-----------------------------
	mpc trajectory solver implementation based on occupancy grid map 
*/

#include <trajectory_planner/mpcPlanner.h>

namespace trajPlanner{
	mpcPlanner::mpcPlanner(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "mpc_planner";
		this->hint_ = "[MPCPlanner]";
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerCallback();
	}

	void mpcPlanner::initParam(){ 
		// planning horizon
		if (not this->nh_.getParam(this->ns_ + "/horizon", this->horizon_)){
			this->horizon_ = 20;
			cout << this->hint_ << ": No planning horizon param. Use default: 20" << endl;
		}
		else{
			cout << this->hint_ << ": Planning horizon is set to: " << this->horizon_ << endl;
		}	

		// mininimum height
		if (not this->nh_.getParam(this->ns_ + "/z_range_min", this->zRangeMin_)){
			this->zRangeMin_ = 0.7;
			cout << this->hint_ << ": No z range min param. Use default: 0.7m" << endl;
		}
		else{
			cout << this->hint_ << ": Z range min is set to: " << this->zRangeMin_ << "m" << endl;
		}

		// maximum height
		if (not this->nh_.getParam(this->ns_ + "/z_range_max", this->zRangeMax_)){
			this->zRangeMax_ = 1.2;
			cout << this->hint_ << ": No z range max param. Use default: 1.2m" << endl;
		}
		else{
			cout << this->hint_ << ": Z range max is set to: " << this->zRangeMax_ << "m" << endl;
		}	

		// pointcloud resolution for clustering
		if (not this->nh_.getParam(this->ns_ + "/cloud_res", this->cloudRes_)){
			this->cloudRes_ = 0.2;
			cout << this->hint_ << ": No cloud res param. Use default: 0.2" << endl;
		}
		else{
			cout << this->hint_ << ": Cloud res is set to: " << this->cloudRes_ << endl;
		}	

		// local cloud region size x 
		if (not this->nh_.getParam(this->ns_ + "/local_cloud_region_x", this->regionSizeX_)){
			this->regionSizeX_ = 5.0;
			cout << this->hint_ << ": No local cloud region size x param. Use default: 5.0m" << endl;
		}
		else{
			cout << this->hint_ << ": Local cloud region size x is set to: " << this->regionSizeX_ << "m" << endl;
		}	

		// local cloud region size y 
		if (not this->nh_.getParam(this->ns_ + "/local_cloud_region_y", this->regionSizeY_)){
			this->regionSizeY_ = 2.0;
			cout << this->hint_ << ": No local cloud region size y param. Use default: 2.0m" << endl;
		}
		else{
			cout << this->hint_ << ": Local cloud region size y is set to: " << this->regionSizeY_ << "m" << endl;
		}	

		// ground height
		if (not this->nh_.getParam(this->ns_ + "/ground_height", this->groundHeight_)){
			this->groundHeight_ = 0.3;
			cout << this->hint_ << ": No ground height param. Use default: 0.3m" << endl;
		}
		else{
			cout << this->hint_ << ": Ground height is set to: " << this->groundHeight_ << "m" << endl;
		}		

		// ceiling height
		if (not this->nh_.getParam(this->ns_ + "/ceiling_height", this->ceilingHeight_)){
			this->ceilingHeight_ = 2.0;
			cout << this->hint_ << ": No ceiling height param. Use default: 2.0m" << endl;
		}
		else{
			cout << this->hint_ << ": Ceiling height is set to: " << this->ceilingHeight_ << "m" << endl;
		}		

		// safety distance
		if (not this->nh_.getParam(this->ns_ + "/static_safety_dist", this->staticSafetyDist_)){
			this->staticSafetyDist_ = 0.5;
			cout << this->hint_ << ": No static safety distance param. Use default: 0.5m" << endl;
		}
		else{
			cout << this->hint_ << ": Static safety distance is set to: " << this->staticSafetyDist_ << "m" << endl;
		}	

		if (not this->nh_.getParam(this->ns_ + "/dynamic_safety_dist", this->dynamicSafetyDist_)){
			this->dynamicSafetyDist_ = 0.5;
			cout << this->hint_ << ": No dynamic safety distance param. Use default: 0.5m" << endl;
		}
		else{
			cout << this->hint_ << ": Dynamic safety distance is set to: " << this->dynamicSafetyDist_ << "m" << endl;
		}	

		// static slack variable
		if (not this->nh_.getParam(this->ns_ + "/static_constraint_slack_ratio", this->staticSlack_)){
			this->staticSlack_ = 0.5;
			cout << this->hint_ << ": No static slack variable param. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": Static slack variable is set to: " << this->staticSlack_ << endl;
		}				

		// dynamic slack variable
		if (not this->nh_.getParam(this->ns_ + "/dynamic_constraint_slack_ratio", this->dynamicSlack_)){
			this->dynamicSlack_ = 0.5;
			cout << this->hint_ << ": No dynamic slack variable param. Use default: 0.5" << endl;
		}
		else{
			cout << this->hint_ << ": Dynamic slack variable is set to: " << this->dynamicSlack_ << endl;
		}

	}

	void mpcPlanner::initModules(){
		this->obclustering_.reset(new obstacleClustering (this->cloudRes_));
	}

	void mpcPlanner::registerPub(){
		this->mpcTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>(this->ns_ + "/mpc_trajectory", 10);
		this->mpcTrajHistVisPub_ = this->nh_.advertise<nav_msgs::Path>(this->ns_ + "/traj_history", 10);
		this->candidateTrajPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/candidate_trajectories", 10);
		this->localCloudPub_ = this->nh_.advertise<sensor_msgs::PointCloud2>(this->ns_ + "/local_cloud", 10);
		this->staticObstacleVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/static_obstacles", 10);
		this->dynamicObstacleVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>(this->ns_ + "/dynamic_obstacles", 10);
		this->facingPub_ = this->nh_.advertise<visualization_msgs::Marker>(this->ns_ + "/clustering_facing", 10);
	}

	void mpcPlanner::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &mpcPlanner::visCB, this);
		this->clusteringTimer_ = this->nh_.createTimer(ros::Duration(0.05), &mpcPlanner::staticObstacleClusteringCB, this);
	}

	void mpcPlanner::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void mpcPlanner::staticObstacleClusteringCB(const ros::TimerEvent&){
		// ros::Time clusteringStartTime = ros::Time::now();
		// cout<<"[MPC Planner]: clustering CB start time "<<clusteringStartTime<<endl;
		if (this->inputTraj_.size() == 0 or not this->stateReceived_) return;
		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		std::vector<Eigen::Vector3d> currCloud;
		double offset = 2.0;
		
		double angle = atan2(this->currVel_(1), this->currVel_(0));		
		if (this->currVel_.norm()>=0.3 or this->firstTime_){
			this->angle_ = angle;
		}
		Eigen::Vector3d faceDirection (cos(this->angle_), sin(this->angle_), 0);

		Eigen::Vector3d sideDirection (-sin(this->angle_), cos(this->angle_), 0); // positive (left side)
		Eigen::Vector3d pOrigin = this->currPos_ - offset * faceDirection;

		// find four vextex of the bounding boxes
		Eigen::Vector3d p1, p2, p3, p4;
		p1 = pOrigin + this->regionSizeY_ * sideDirection;
		p2 = pOrigin - this->regionSizeY_ * sideDirection;
		p3 = p1 + (this->regionSizeX_ + offset) * faceDirection;
		p4 = p2 + (this->regionSizeX_ + offset) * faceDirection;

		double xStart = floor(std::min({p1(0), p2(0), p3(0), p4(0)})/this->cloudRes_)*this->cloudRes_;
		double xEnd = ceil(std::max({p1(0), p2(0), p3(0), p4(0)})/this->cloudRes_)*this->cloudRes_;
		double yStart = floor(std::min({p1(1), p2(1), p3(1), p4(1)})/this->cloudRes_)*this->cloudRes_;
		double yEnd = ceil(std::max({p1(1), p2(1), p3(1), p4(1)})/this->cloudRes_)*this->cloudRes_;

		for (double ix=xStart; ix<=xEnd; ix+=this->cloudRes_){
			for (double iy=yStart; iy<=yEnd; iy+=this->cloudRes_){
				for (double iz=this->groundHeight_; iz<=this->ceilingHeight_; iz+=this->cloudRes_){
					Eigen::Vector3d p (ix, iy, iz);
					if ((p - pOrigin).dot(faceDirection) >= 0){
						if (this->map_->isInMap(p) and this->map_->isInflatedOccupied(p)){
							currCloud.push_back(p);
						}
					}
				}
			}
		}	
		this->currCloud_ = currCloud;
		this->obclustering_->run(currCloud);
		this->refinedBBoxVertices_ = this->obclustering_->getRefinedBBoxes();
		// ros::Time clusteringEndTime = ros::Time::now();
		// cout<<"[MPC Planner]: clustering time: "<<(clusteringEndTime-clusteringStartTime).toSec()<<endl;
	}

	void mpcPlanner::updateMaxVel(double maxVel){
		this->maxVel_ = maxVel;
	}

	void mpcPlanner::updateMaxAcc(double maxAcc){
		this->maxAcc_ = maxAcc;
	}

	void mpcPlanner::updateCurrStates(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel){
		this->currPos_ = pos;
		this->currVel_ = vel;
		this->trajHist_.push_back(pos);
		this->numHalfSpace_ = 0;
		this->stateReceived_ = true;
	}

	void mpcPlanner::updateCurrStates(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel, const double &yaw){
		this->currPos_ = pos;
		this->currVel_ = vel;
		this->currYaw_ = yaw;
		this->trajHist_.push_back(pos);
		this->updateFovParam();
		this->stateReceived_ = true;
	}

	void mpcPlanner::updateFovParam(){
		double maxAngle = this->currYaw_-87/2*M_PI/180.0;
		double minAngle = this->currYaw_+87/2*M_PI/180.0;

		Eigen::Vector3d half1, half2;
		double a1, b1, c1;
		double a2, b2, c2;

		a1 = sin(maxAngle);
		b1 = -cos(maxAngle);
		c1 = a1 * this->currPos_(0) + b1 * this->currPos_(1);
		half1<<a1, b1, c1;
		
		a2 = sin(minAngle);
		b2 = -cos(minAngle);
		c2 = a2 * this->currPos_(0) + b2 * this->currPos_(1);
		half2<<a2, b2, c2;

		this->halfMax_ = half1;
		this->halfMin_ = half2;
		this->numHalfSpace_ = 2;
	}

	void mpcPlanner::updatePath(const nav_msgs::Path& path, double ts){
		std::vector<Eigen::Vector3d> pathTemp;
		for (int i=0; i<(path.poses.size()); ++i){
			Eigen::Vector3d p (path.poses[i].pose.position.x, path.poses[i].pose.position.y, path.poses[i].pose.position.z);
			pathTemp.push_back(p); 
		}
		this->updatePath(pathTemp, ts);

	}

	void mpcPlanner::updatePath(const std::vector<Eigen::Vector3d>& path, double ts){
		this->ts_ = ts;
		this->inputTraj_ = path;
		this->firstTime_ = true;
		this->stateReceived_ = false;
		this->trajHist_.clear();
		this->lastRefStartIdx_ = 0;
	}

	void mpcPlanner::updateDynamicObstacles(const std::vector<Eigen::Vector3d>& obstaclesPos, const std::vector<Eigen::Vector3d>& obstaclesVel, const std::vector<Eigen::Vector3d>& obstaclesSize){
		std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesPosTemp;
		std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesSizeTemp;
		std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesVelTemp;
		dynamicObstaclesPosTemp.clear();
		dynamicObstaclesSizeTemp.clear();
		dynamicObstaclesVelTemp.clear();

		dynamicObstaclesPosTemp.resize(obstaclesPos.size());
		dynamicObstaclesSizeTemp.resize(obstaclesPos.size());
		dynamicObstaclesVelTemp.resize(obstaclesPos.size());
		for (int i=0; i<int(obstaclesPos.size()); ++i){
			Eigen::Vector3d pos = obstaclesPos[i];
            Eigen::Vector3d size = obstaclesSize[i];
			Eigen::Vector3d vel = obstaclesVel[i];
			for (int j=0; j<this->horizon_; j++){
				dynamicObstaclesPosTemp[i].push_back(pos);
				dynamicObstaclesSizeTemp[i].push_back(size);
				dynamicObstaclesVelTemp[i].push_back(vel);
			}
        }

		this->dynamicObstaclesPos_ = dynamicObstaclesPosTemp;
		this->dynamicObstaclesSize_ = dynamicObstaclesSizeTemp;
		this->dynamicObstaclesVel_ = dynamicObstaclesVelTemp;
	}

	void mpcPlanner::updatePredObstacles(const std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predPos, const std::vector<std::vector<std::vector<Eigen::Vector3d>>> &predSize, const std::vector<Eigen::VectorXd> &intentProb){
		if (predPos.size()){
			std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesPosTemp;
			std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesSizeTemp;
			dynamicObstaclesPosTemp.clear();
			dynamicObstaclesSizeTemp.clear();
			dynamicObstaclesPosTemp.resize(predPos.size());
			dynamicObstaclesSizeTemp.resize(predPos.size());
			for (int i=0;i<int(predPos.size());i++){
				Eigen::Vector3d pos, size;
				pos = predPos[i][0][0];
				size = predSize[i][0][0];
				for (int j=0; j<this->horizon_; j++){
					dynamicObstaclesPosTemp[i].push_back(pos);
					dynamicObstaclesSizeTemp[i].push_back(size);
				}
			}
			this->dynamicObstaclesPos_ = dynamicObstaclesPosTemp;
			this->dynamicObstaclesSize_ = dynamicObstaclesSizeTemp;
			this->obPredPos_ = predPos;
			this->obPredSize_ = predSize;
			this->obIntentProb_ = intentProb;
		}
		else{
			this->dynamicObstaclesPos_.clear();
			this->dynamicObstaclesSize_.clear();
			this->obPredPos_.clear();
			this->obPredSize_.clear();
			this->obIntentProb_.clear();
		}
	}

bool mpcPlanner::solveTraj(const std::vector<staticObstacle> &staticObstacles, const std::vector<std::vector<Eigen::Vector3d>> &dynamicObstaclesPos, const std::vector<std::vector<Eigen::Vector3d>> &dynamicObstaclesSize, 
	std::vector<Eigen::VectorXd> &statesSol, std::vector<Eigen::VectorXd> &controlsSol, std::vector<Eigen::Matrix<double, numStates, 1>> &xRef, const double &timeLimit){
	// set the preview window
	if (this->firstTime_){
		this->currentStatesSol_.clear();
		this->currentControlsSol_.clear();
	}
    const int mpcWindow = this->horizon_-1;
	int numObs;
	int numHalfSpace = this->numHalfSpace_;

    // // allocate the dynamics matrices
    Eigen::Matrix<double, numStates, numStates> a;
    Eigen::Matrix<double, numStates, numControls> b;

    // // allocate the constraints vector
    Eigen::Matrix<double, numStates, 1> xMax;
    Eigen::Matrix<double, numStates, 1> xMin;
    Eigen::Matrix<double, numControls, 1> uMax;
    Eigen::Matrix<double, numControls, 1> uMin;

    // allocate the weight matrices
    Eigen::DiagonalMatrix<double, numStates> Q;
    Eigen::DiagonalMatrix<double, numControls> R;

    // allocate the initial and the reference state space
    Eigen::Matrix<double, numStates, 1> x0;
	x0.setZero();
	x0(0,0) = this->currPos_(0);
	x0(1,0) = this->currPos_(1);
	x0(2,0) = this->currPos_(2);
	x0(3,0) = this->currVel_(0);
	x0(4,0) = this->currVel_(1);
	x0(5,0) = this->currVel_(2);
    // allocate QP problem matrices and vectores
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient;
    Eigen::SparseMatrix<double> constraintMatrix;
    Eigen::Matrix<double, Eigen::Dynamic, 1> lowerBound;
    Eigen::Matrix<double, Eigen::Dynamic, 1> upperBound;

	std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> oxyz;
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> osize;
	std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> yaw;
	std::vector<std::vector<int>> isDynamic;
	// std::vector<staticObstacle> staticObstacles = this->obclustering_->getStaticObstacles();
	// std::vector<staticObstacle> staticObstacles;
	updateObstacleParam(staticObstacles, dynamicObstaclesPos, dynamicObstaclesSize, numObs, mpcWindow, oxyz, osize, yaw, isDynamic);

    // set MPC problem quantities
    setDynamicsMatrices(a, b);
    setInequalityConstraints(xMax, xMin, uMax, uMin);
    setWeightMatrices(Q, R);

    // cast the MPC problem as QP problem
    castMPCToQPHessian(Q, R, mpcWindow, hessian);
    castMPCToQPGradient(Q, xRef, mpcWindow, gradient);
    // castMPCToQPConstraintMatrix(a, b, mpcWindow, linearMatrix);
	castMPCToQPConstraintMatrix(a, b,constraintMatrix, numObs, mpcWindow, oxyz, osize, yaw, isDynamic);
    castMPCToQPConstraintVectors(xMax, xMin, uMax, uMin, x0, lowerBound, upperBound, numObs,mpcWindow,oxyz,osize,yaw);
    // // instantiate the solver
    OsqpEigen::Solver solver;
	// OSQPWrapper::OptimizatorSolver solver;

    // // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
	if (not this->firstTime_){
		solver.settings()->setTimeLimit(timeLimit);
	}
	// solver.settings()->setAlpha(1.8);
	// solver.settings()->setDualInfeasibilityTolerance(1e-3);
	// solver.settings()->setDualInfeasibilityTollerance();
	// solver.settings()->setAdaptiveRho()
    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(numStates * (mpcWindow + 1) + numControls * mpcWindow);
    // solver.data()->setNumberOfConstraints(numStates * (mpcWindow + 1) + 3*(mpcWindow+1)+ numControls * mpcWindow + numOb*mpcWindow);
    solver.data()->setNumberOfConstraints(numStates * (mpcWindow + 1)+numStates * (mpcWindow + 1)+numControls*mpcWindow + numHalfSpace * mpcWindow +numObs*mpcWindow);
	if (!solver.data()->setHessianMatrix(hessian))
        return 0;
    if (!solver.data()->setGradient(gradient))
        return 0;
    if (!solver.data()->setLinearConstraintsMatrix(constraintMatrix))
        return 0;
    if (!solver.data()->setLowerBound(lowerBound))
        return 0;
    if (!solver.data()->setUpperBound(upperBound))
        return 0;

    // instantiate the solver
    if (!solver.initSolver())
        return 0;
    // controller input and QPSolution vector
    // Eigen::Vector4d ctr;
    Eigen::VectorXd QPSolution;
	Eigen::VectorXd control;
	Eigen::VectorXd state;

	Eigen::VectorXd primalVariable;
	Eigen::VectorXd dualVariable;
	dualVariable.setZero(numStates * (mpcWindow + 1)+numStates * (mpcWindow + 1)+numControls*mpcWindow + numHalfSpace * mpcWindow +numObs*mpcWindow);
	primalVariable.setZero(numStates * (mpcWindow + 1) + numControls * mpcWindow);
	for (int i=0;i<mpcWindow+1;i++){
		if (not this->firstTime_){
			primalVariable.block(numStates*i,0,numStates,1) = this->currentStatesSol_[i];
		}
		else{
			Eigen::VectorXd stateGuess;
			stateGuess.setZero(numStates);
			primalVariable.block(numStates*i,0,numStates,1) = stateGuess;
		}
	}
	for(int i=0;i<mpcWindow;i++){
		if (not this->firstTime_){
			primalVariable.block(numStates*(mpcWindow+1)+numControls*i, 0, numControls, 1) = this->currentControlsSol_[i];
		}
		else{
			Eigen::VectorXd controlGuess;
			controlGuess.setZero(numControls);
			primalVariable.block(numStates*(mpcWindow+1)+numControls*i, 0, numControls, 1) = controlGuess;
		}
	}
	solver.setWarmStart(primalVariable, dualVariable);
	// }
	// solve the QP problem
	if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
		return 0;

	// if (solver.workspace()->info->solve_time>0.015){
	// 	return 0;
	// }

	QPSolution = solver.getSolution();	
	solver.clearSolver();
	controlsSol.clear();
	statesSol.clear();			
	for (int i=0;i<mpcWindow+1;i++){
			state = QPSolution.block(numStates*i, 0, numStates, 1);
			statesSol.push_back(state);
		}
	for (int i=0;i<mpcWindow;i++){
			control = QPSolution.block(numStates*(mpcWindow+1)+numControls*i, 0, numControls, 1);
			controlsSol.push_back(control);
		}

	// this->firstTime_ = false;
    return 1;
}

	bool mpcPlanner::makePlan(){
		if (this->firstTime_){
			this->currentStatesSol_.clear();
			this->currentControlsSol_.clear();
			this->ref_.clear();
		}
		std::vector<Eigen::VectorXd> currentStatesSol;
		std::vector<Eigen::VectorXd> currentControlsSol;
		std::vector<staticObstacle> staticObstacles = this->obclustering_->getStaticObstacles();
		std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesPos = this->dynamicObstaclesPos_;
		std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesSize = this->dynamicObstaclesSize_;
		if(this->firstTime_){
			staticObstacles.clear();
			dynamicObstaclesPos.clear();
			dynamicObstaclesSize.clear();
		}
		std::vector<Eigen::Matrix<double, numStates, 1>> xRef;
		this->getXRef(xRef);
		bool successSolve = this->solveTraj(staticObstacles, dynamicObstaclesPos, dynamicObstaclesSize, currentStatesSol, currentControlsSol, xRef);
		if (successSolve){
			this->currentStatesSol_ = currentStatesSol;
			this->currentControlsSol_ = currentControlsSol;
			this->firstTime_ = false;
			this->ref_ = xRef;
		}
		return successSolve;
	}

	bool mpcPlanner::makePlanWithPred(){
		ros::Time startTime = ros::Time::now();
		std::vector<std::vector<Eigen::VectorXd>> candidateStatesTemp;
		std::vector<std::vector<Eigen::VectorXd>> candidateControlsTemp;
		std::vector<Eigen::Vector3d> trajScore;
		std::vector<int> intentType; 
		if (this->firstTime_){
			this->candidateStates_.clear();
			this->candidateControls_.clear();
			this->trajWeightedScore_.clear();
			this->trajScore_.clear();
			this->currentStatesSol_.clear();
			this->currentControlsSol_.clear();
			this->ref_.clear();
		}
		int obIdx;
		std::vector<std::vector<std::vector<Eigen::Vector3d>>> obstaclesPosComb;
		std::vector<std::vector<std::vector<Eigen::Vector3d>>> obstaclesSizeComb;
		bool validTraj;
		std::vector<staticObstacle> staticObstacles;
		std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesPos;
		std::vector<std::vector<Eigen::Vector3d>> dynamicObstaclesSize;
		if (not this->firstTime_){
			staticObstacles = this->obclustering_->getStaticObstacles();
			dynamicObstaclesPos = this->dynamicObstaclesPos_;
			dynamicObstaclesSize = this->dynamicObstaclesSize_;
		}
		else{//don't linearize constraints for the first time
			staticObstacles.clear();
			dynamicObstaclesPos.clear();
			dynamicObstaclesSize.clear();
		}

		std::vector<Eigen::Matrix<double, numStates, 1>> xRef;
		this->getXRef(xRef);
		if (this->obPredPos_.size() and not this->firstTime_){
			this->getIntentComb(obIdx, obstaclesPosComb, obstaclesSizeComb, xRef);
			bool successSolve;
			for (int i=0; i<int(obstaclesPosComb.size());i++){
				std::vector<Eigen::VectorXd> statesSol;
				std::vector<Eigen::VectorXd> controlsSol;
				double time = (ros::Time::now()-startTime).toSec();
				if (time<0.1){
					double timeLimit = max(0.03 - time, 0.02);
					successSolve = this->solveTraj(staticObstacles, obstaclesPosComb[i], obstaclesSizeComb[i], statesSol, controlsSol, xRef, timeLimit);
					if (successSolve){
						candidateStatesTemp.push_back(statesSol);
						candidateControlsTemp.push_back(controlsSol);
						Eigen::Vector3d score;
						score = this->getTrajectoryScore(statesSol, controlsSol, staticObstacles, obstaclesPosComb[i], obstaclesSizeComb[i], xRef);
						trajScore.push_back(score);
						intentType.push_back(i);
					}
				}
				else{
					break;
				}
			}
			this->candidateStates_ = candidateStatesTemp;
			this->candidateControls_ = candidateControlsTemp;
			if (this->candidateStates_.size()){
				this->firstTime_ = false;
				validTraj = true;
				// get best traj
				int bestTrajIdx = this->evaluateTraj(trajScore, obIdx, intentType);
				this->currentStatesSol_ = this->candidateStates_[bestTrajIdx];
				this->currentControlsSol_ = this->candidateControls_[bestTrajIdx];
				this->trajScore_ = trajScore;
				this->ref_ = xRef;
			}
			else{
				validTraj = false;
			}
		}
		else{
			this->candidateStates_.clear();
			this->candidateControls_.clear();
			this->trajWeightedScore_.clear();
			this->trajScore_.clear();
			std::vector<Eigen::VectorXd> currentStatesSol;
			std::vector<Eigen::VectorXd> currentControlsSol;
			validTraj = this->solveTraj(staticObstacles, dynamicObstaclesPos, dynamicObstaclesSize, currentStatesSol, currentControlsSol, xRef);
			if (validTraj){
				this->currentStatesSol_ = currentStatesSol;
				this->currentControlsSol_ = currentControlsSol;
				this->firstTime_ = false;
				this->ref_ = xRef;
			}
		}
		return validTraj;
	}

	void mpcPlanner::findClosestObstacle(int &obIdx, const std::vector<Eigen::Matrix<double, numStates, 1>> &xRef){
		obIdx = -1;
		double minDist = INFINITY;
		if (this->firstTime_){
			for (int i=0;i<int(this->dynamicObstaclesPos_.size());i++){
				double dist = (this->currPos_-this->dynamicObstaclesPos_[i][0]).norm();
				if (dist < minDist){
					minDist = dist;
					obIdx = i;
				}
			}
		}
		else{
			for (int i=0;i<int(this->dynamicObstaclesPos_.size());i++){
				double dist = 0;
				for (int j=0;j<int(this->currentStatesSol_.size()/3);j++){
					Eigen::Vector3d state (this->currentStatesSol_[0](0), this->currentStatesSol_[0](1), this->currentStatesSol_[0](2));
					Eigen::Vector3d nextState(this->currentStatesSol_[1](0), this->currentStatesSol_[1](1), this->currentStatesSol_[1](2));
					double trajDirectionAngle = atan2(nextState(1)-state(1), nextState(0)-state(0));
					double obsDirectionAngle = atan2(this->dynamicObstaclesPos_[i][0](1)-state(1), this->dynamicObstaclesPos_[i][0](0)-state(0));
					double weight = exp(-j); //higher weight for closer time
					double d = (state-this->dynamicObstaclesPos_[i][0]).norm();
					const double a = 3.0; //larger a means less importance for obstacles backward
					dist += weight * d * (a-cos(trajDirectionAngle - obsDirectionAngle));
					if (dist > minDist){
						break;
					}
				}
				if (dist < minDist){
					minDist = dist;
					obIdx = i;
				}
			}
		}
	}

	void mpcPlanner::getIntentComb(int &obIdx, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &intentCombPos, std::vector<std::vector<std::vector<Eigen::Vector3d>>> &intentCombSize, const std::vector<Eigen::Matrix<double, numStates, 1>> &xRef){
		intentCombPos.clear();
		intentCombSize.clear();
		this->findClosestObstacle(obIdx, xRef);
		this->obIdx_ = obIdx;
		intentCombPos.resize(6);
		intentCombSize.resize(6);
		std::vector<std::vector<std::vector<Eigen::Vector3d>>> intentCombPosTemp;
		std::vector<std::vector<std::vector<Eigen::Vector3d>>> intentCombSizeTemp;
		intentCombPosTemp.resize(6);
		intentCombSizeTemp.resize(6);

		std::vector<std::pair<double, int>> weight = {std::make_pair(this->obIntentProb_[obIdx](dynamicPredictor::STOP),0),
			std::make_pair(this->obIntentProb_[obIdx](dynamicPredictor::LEFT), 1),
			std::make_pair(this->obIntentProb_[obIdx](dynamicPredictor::RIGHT), 2),
			std::make_pair(this->obIntentProb_[obIdx](dynamicPredictor::FORWARD), 3),
			std::make_pair(std::max(this->obIntentProb_[obIdx](dynamicPredictor::LEFT),this->obIntentProb_[obIdx](dynamicPredictor::FORWARD)), 4),
			std::make_pair(std::max(this->obIntentProb_[obIdx](dynamicPredictor::RIGHT),this->obIntentProb_[obIdx](dynamicPredictor::FORWARD)), 5)};
		std::sort(weight.begin(), weight.end());  

		// for closest obstacle, find 6 intent combinations
		intentCombPosTemp[0].push_back(this->obPredPos_[obIdx][dynamicPredictor::STOP]);
		intentCombPosTemp[1].push_back(this->obPredPos_[obIdx][dynamicPredictor::LEFT]);
		intentCombPosTemp[2].push_back(this->obPredPos_[obIdx][dynamicPredictor::RIGHT]);
		intentCombPosTemp[3].push_back(this->obPredPos_[obIdx][dynamicPredictor::FORWARD]);

		intentCombPosTemp[4].push_back(this->obPredPos_[obIdx][dynamicPredictor::LEFT]);
		intentCombPosTemp[4].push_back(this->obPredPos_[obIdx][dynamicPredictor::FORWARD]);

		intentCombPosTemp[5].push_back(this->obPredPos_[obIdx][dynamicPredictor::RIGHT]);
		intentCombPosTemp[5].push_back(this->obPredPos_[obIdx][dynamicPredictor::FORWARD]);

		intentCombSizeTemp[0].push_back(this->obPredSize_[obIdx][dynamicPredictor::STOP]);
		intentCombSizeTemp[1].push_back(this->obPredSize_[obIdx][dynamicPredictor::LEFT]);
		intentCombSizeTemp[2].push_back(this->obPredSize_[obIdx][dynamicPredictor::RIGHT]);
		intentCombSizeTemp[3].push_back(this->obPredSize_[obIdx][dynamicPredictor::FORWARD]);

		intentCombSizeTemp[4].push_back(this->obPredSize_[obIdx][dynamicPredictor::LEFT]);
		intentCombSizeTemp[4].push_back(this->obPredSize_[obIdx][dynamicPredictor::FORWARD]);

		intentCombSizeTemp[5].push_back(this->obPredSize_[obIdx][dynamicPredictor::RIGHT]);
		intentCombSizeTemp[5].push_back(this->obPredSize_[obIdx][dynamicPredictor::FORWARD]);

		for (int i=0;i<6;i++){
			intentCombPos[i] = intentCombPosTemp[weight[5-i].second];
			intentCombSize[i] = intentCombSizeTemp[weight[5-i].second];
		}

		// for rest of obstacles, use maximum intent
		for (int i=0;i<int(intentCombPos.size());i++){
			for (int j=0;j<int(this->obPredPos_.size());j++){
				if (j != this->obIdx_){
					int maxIntent;
					double maxProb = this->obIntentProb_[j].maxCoeff(&maxIntent);
					intentCombPos[i].push_back(this->obPredPos_[j][maxIntent]);
					intentCombSize[i].push_back(this->obPredSize_[j][maxIntent]);
				}
			}
		}	
	}

	Eigen::Vector3d mpcPlanner::getTrajectoryScore(const std::vector<Eigen::VectorXd> &states, const std::vector<Eigen::VectorXd> &controls, const std::vector<staticObstacle> &staticObstacles, const std::vector<std::vector<Eigen::Vector3d>> &obstaclePos, const std::vector<std::vector<Eigen::Vector3d>> &obstacleSize, const std::vector<Eigen::Matrix<double, numStates, 1>> &xRef){
		Eigen::Vector3d score;
		double consistencyScore = this->getConsistencyScore(states);
		double detourScore = this->getDetourScore(states, xRef);
		double saftyScore = this->getSafetyScore(states, staticObstacles, obstaclePos, obstacleSize);
		score<<consistencyScore, detourScore, saftyScore;
		return score;
	}

	double mpcPlanner::getConsistencyScore(const std::vector<Eigen::VectorXd> &state){
		int numConsistencyStep = 10;
		if (this->firstTime_){
			return 0;
		}
		else{
			double totalDist = 0;
			for (int i=0;i<numConsistencyStep;i++){
				Eigen::Vector3d prevPos;
				prevPos<<this->currentStatesSol_[i](0), this->currentStatesSol_[i](1), this->currentStatesSol_[i](2);
				Eigen::Vector3d pos;
				pos<<state[i](0), state[i](1), state[i](2);
				totalDist += (prevPos-pos).norm();
			}
			totalDist /= numConsistencyStep;
			totalDist = max(totalDist, 0.1);
			return totalDist;
		}
	}

	double mpcPlanner::getDetourScore(const std::vector<Eigen::VectorXd> &state, const std::vector<Eigen::Matrix<double, numStates, 1>> &ref){
		double totalDist = 0;
		for (int i=0;i<int(state.size());i++){
			Eigen::Vector3d refPos;
			refPos<<ref[i](0,0), ref[i](1,0), ref[i](2,0);
			Eigen::Vector3d pos;
			pos<<state[i](0), state[i](1), state[i](2);
			totalDist += (refPos-pos).norm();
		}
		totalDist /= state.size();
		totalDist = max(totalDist, 0.1);
		return totalDist;
	}

	double mpcPlanner::getSafetyScore(const std::vector<Eigen::VectorXd> &state, const std::vector<staticObstacle> &staticObstacles, const std::vector<std::vector<Eigen::Vector3d>> &obstaclePos, const std::vector<std::vector<Eigen::Vector3d>> &obstacleSize){
		double totalDist = 0;
		for (int i=0;i<int(state.size());i++){
			double dist = 0;
			double totalWeight = 0;
			Eigen::Vector3d pos;
			pos<<state[i](0),state[i](1),0;
			for (int j=0;j<int(obstaclePos.size());j++){
				Eigen::Vector3d dynamicObPos = obstaclePos[j][i];
				dynamicObPos(2) = 0;
				double maxSize = sqrt(pow(obstacleSize[j][i](0),2)+pow(obstacleSize[j][i](1),2));
				double d = (pos-dynamicObPos).norm();
				double weight = (1-tanh(atanh(0.5)/(this->dynamicSafetyDist_+maxSize)*d));

				dist += d*weight;
				totalWeight += weight;
			}
			for (int j=0;j<int(staticObstacles.size());j++){
				Eigen::Vector3d statObPos(staticObstacles[j].centroid(0), staticObstacles[j].centroid(1), 0);
				double maxSize = sqrt(pow(staticObstacles[j].size(0)/2,2)+pow(staticObstacles[j].size(1)/2,2));
				double d = (pos - statObPos).norm();
				double weight = (1-tanh(atanh(0.5)/(this->staticSafetyDist_+maxSize)*d));

				dist += d*weight;
				totalWeight += weight;
			}
			dist /= totalWeight;
			totalDist += dist;
		}
		totalDist /= int(state.size());
		return totalDist;

	}

	int mpcPlanner::evaluateTraj(std::vector<Eigen::Vector3d> &trajScore, const int &obIdx, const std::vector<int> &intentType){
		this->trajWeightedScore_.clear();
		std::vector<double> consistentScore, detourScore, safetyScore;
		for (int i=0;i<int(trajScore.size());i++){
			Eigen::Vector3d score = trajScore[i];
			consistentScore.push_back(score(0));
			detourScore.push_back(score(1));
			safetyScore.push_back(score(2));
		}
		// remap score
		double consistentAvg = std::accumulate(consistentScore.begin(), consistentScore.end(), 0.0)/consistentScore.size();
		double detourAvg = std::accumulate(detourScore.begin(), detourScore.end(),0.0)/detourScore.size();
		double safetyAvg = std::accumulate(safetyScore.begin(), safetyScore.end(), 0.0)/safetyScore.size();
		for (int i=0;i<int(consistentScore.size());i++){
			consistentScore[i] = consistentAvg/consistentScore[i];
			detourScore[i] = detourAvg/detourScore[i];
			safetyScore[i] = safetyScore[i]/safetyAvg;
		}
		Eigen::VectorXd weight;
		weight.resize(6);
		weight<<this->obIntentProb_[obIdx](dynamicPredictor::STOP), this->obIntentProb_[obIdx](dynamicPredictor::LEFT), 
				this->obIntentProb_[obIdx](dynamicPredictor::RIGHT), this->obIntentProb_[obIdx](dynamicPredictor::FORWARD),
				std::max(this->obIntentProb_[obIdx](dynamicPredictor::LEFT), this->obIntentProb_[obIdx](dynamicPredictor::FORWARD)),
				std::max(this->obIntentProb_[obIdx](dynamicPredictor::RIGHT), this->obIntentProb_[obIdx](dynamicPredictor::FORWARD));


		Eigen::VectorXd weightedScore;
		weightedScore.resize(consistentScore.size());
		for (int i=0;i<int(consistentScore.size());i++){
			weightedScore(i) = weight(intentType[i])*(1.0*consistentScore[i] + 1.0*detourScore[i] + 1.0*safetyScore[i]);
			this->trajWeightedScore_.push_back(weightedScore(i));
		}
		int bestTrajIdx;
		int maxWeightIdx;
		double s = weightedScore.maxCoeff(&bestTrajIdx);
		double w = weight.maxCoeff(&maxWeightIdx);
		return bestTrajIdx;
	}



	void mpcPlanner::setDynamicsMatrices(Eigen::Matrix<double, numStates, numStates> &A, Eigen::Matrix<double, numStates, numControls> &B){
		A.setZero();
		A.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
		A.block(0, 3, 3, 3) = Eigen::Matrix3d::Identity() * this->ts_;
		A.block(3, 3, 3, 3) = Eigen::Matrix3d::Identity();

		B.setZero();
		B.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity() * 1/2 * pow(this->ts_, 2);
		B.block(3, 0, 3, 3) = Eigen::Matrix3d::Identity() * this->ts_;
		B.block(6, 3, 2, 2) = Eigen::Matrix2d::Identity();
	}


	void mpcPlanner::setInequalityConstraints(Eigen::Matrix<double, numStates, 1> &xMax, Eigen::Matrix<double, numStates, 1> &xMin,
								Eigen::Matrix<double, numControls, 1> &uMax, Eigen::Matrix<double, numControls, 1> &uMin){
		// state bound
		xMin <<  -INFINITY, -INFINITY, this->zRangeMin_, -this->maxVel_, -this->maxVel_, -this->maxVel_, -INFINITY, -INFINITY;
		xMax << INFINITY, INFINITY, this->zRangeMax_, this->maxVel_, this->maxVel_, this->maxVel_, INFINITY, INFINITY;

		// control bound
		double skslimit = 1.0 - pow((1 - this->staticSlack_), 2);
		double skdlimit = 1.0 - pow((1 - this->dynamicSlack_), 2);
		uMin << -this->maxAcc_, -this->maxAcc_, -this->maxAcc_, 0.0, 0.0;
		uMax << this->maxAcc_, this->maxAcc_, this->maxAcc_, skdlimit, skslimit;
	}



	void mpcPlanner::setWeightMatrices(Eigen::DiagonalMatrix<double,numStates> &Q, Eigen::DiagonalMatrix<double,numControls> &R){
		Q.diagonal() << 1000.0, 1000.0, 1000.0, 0, 0, 0, 100.0, 1000.0;
		R.diagonal() << 10.0, 10.0, 10.0, 1.0, 1.0;
	}
	void mpcPlanner::castMPCToQPHessian(const Eigen::DiagonalMatrix<double,numStates> &Q, const Eigen::DiagonalMatrix<double,numControls> &R, int mpcWindow, Eigen::SparseMatrix<double>& hessianMatrix){
		hessianMatrix.resize(numStates * (mpcWindow + 1) + numControls * mpcWindow,
							numStates * (mpcWindow + 1) + numControls * mpcWindow);

		// populate hessian matrix
		for (int i = 0; i < numStates * (mpcWindow + 1) + numControls * mpcWindow; i++){
			if (i < numStates * (mpcWindow + 1)){
				int posQ = i % numStates;
				float value = Q.diagonal()[posQ];
				if (value != 0)
					hessianMatrix.insert(i, i) = value;
			} 
			else{
				int posR = i % numControls;
				float value = R.diagonal()[posR];
				if (value != 0)
					hessianMatrix.insert(i, i) = value;
			}
		}
	}
	void mpcPlanner::castMPCToQPGradient(const Eigen::DiagonalMatrix<double,numStates> &Q, const std::vector<Eigen::Matrix<double, numStates, 1>>& xRef, int mpcWindow, Eigen::VectorXd& gradient){
		std::vector<Eigen::Matrix<double, numStates, 1>> Qx_ref;
		for (int i = 0; i < xRef.size(); i++){
			Eigen::Matrix<double, numStates, 1> ref = Q * (-xRef[i]);
			Qx_ref.push_back(ref);
		}
		// populate the gradient vector
		gradient = Eigen::VectorXd::Zero(numStates * (mpcWindow + 1) + numControls * mpcWindow, 1);
		for (int i = 0; i < (mpcWindow + 1); i++){
			for (int j = 0; j < numStates; j++){
				double value = Qx_ref[i](j, 0);
				gradient(i*numStates+j, 0) = value;
			}
		}
	}

	void mpcPlanner::getXRef(std::vector<Eigen::Matrix<double, numStates, 1>>& xRef){
		std::vector<Eigen::Vector3d> referenceTraj;
		this->getReferenceTraj(referenceTraj);
		std::vector<Eigen::Matrix<double, numStates, 1>> xRefTemp;
		Eigen::Matrix<double, numStates, 1> ref;
		ref.setZero();
		for (int i = 0; i<int(referenceTraj.size()); ++i){
			ref(0,0) = referenceTraj[i](0);
			ref(1,0) = referenceTraj[i](1);
			ref(2,0) = referenceTraj[i](2);
			xRefTemp.push_back(ref);
		}
		xRef = xRefTemp;
	}


	void mpcPlanner::castMPCToQPConstraintMatrix(Eigen::Matrix<double, numStates, numStates> &A, Eigen::Matrix<double, numStates, numControls> &B, 
		Eigen::SparseMatrix<double> &constraintMatrix, int numObs, int mpcWindow, 
		std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw, 
		std::vector<std::vector<int>> &isDyamic){
		int numHalfSpace = this->numHalfSpace_;
		constraintMatrix.resize(numStates * (mpcWindow+1) + numStates * (mpcWindow+1) + numControls * mpcWindow  + numHalfSpace * mpcWindow + numObs * mpcWindow,
								numStates * (mpcWindow+1) + numControls * mpcWindow);
		
		// populate linear constraint matrix
		//equality
		for (int i = 0; i < numStates * (mpcWindow + 1); i++)
		{
			constraintMatrix.insert(i, i) = -1;
		}

		for (int i = 0; i < mpcWindow; i++)
			for (int j = 0; j < numStates; j++)
				for (int k = 0; k < numStates; k++)
				{
					float value = A(j, k);
					if (value != 0)
					{
						constraintMatrix.insert(numStates * (i + 1) + j, numStates * i + k) = value;
					}
				}

		for (int i = 0; i < mpcWindow; i++)
			for (int j = 0; j < numStates; j++)
				for (int k = 0; k < numControls; k++)
				{
					float value = B(j, k);
					if (value != 0)
					{
						constraintMatrix.insert(numStates * (i + 1) + j, numControls * i + k + numStates * (mpcWindow + 1))
							= value;
					}
				}

		//inequality
		for (int i = 0; i < numStates * (mpcWindow + 1) + numControls * mpcWindow; i++)
		{
			constraintMatrix.insert(i + (mpcWindow + 1) * numStates, i) = 1;
		}
		if (this->numHalfSpace_){
			for (int i=0; i < mpcWindow; i++){
				// for (int j=0;j < numHalfSpace; j++){
					constraintMatrix.insert(numHalfSpace * i + 0 + numStates * (mpcWindow + 1) + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates * i + 0) = this->halfMax_(0);
					constraintMatrix.insert(numHalfSpace * i + 0 + numStates * (mpcWindow + 1) + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates * i + 1) = this->halfMax_(1);
					constraintMatrix.insert(numHalfSpace * i + 1 + numStates * (mpcWindow + 1) + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates * i + 0) = this->halfMin_(0);
					constraintMatrix.insert(numHalfSpace * i + 1 + numStates * (mpcWindow + 1) + numStates * (mpcWindow + 1) + numControls * mpcWindow, numStates * i + 1) = this->halfMin_(1);
				
				// }

			}
		}

		for (int i = 0; i < mpcWindow; i++){
			double cx, cy, cz;
			if (this->currentStatesSol_.size()!=0){
				cx = this->currentStatesSol_[i](0);
				cy = this->currentStatesSol_[i](1);
				cz = this->currentStatesSol_[i](2);
			}
			else{
				cx = this->currPos_(0);
				cy = this->currPos_(1);
				cz = this->currPos_(2);
			}
			for (int j = 0; j < numObs; j++){
				double fxx,fyy,fzz;
				// fxyz = pow((cx-oxyz(j,0))*cos(yaw(j,0))+(cy-oxyz(j,1))*sin(yaw(j,0)), 2)/pow(osize(j,0),2) + pow(-(cx-oxyz(j,0))*sin(yaw(j,0))+(cy-oxyz(j,1))*cos(yaw(j,0)), 2)/pow(osize(j,1),2) + pow((cz-oxyz(j,2)), 2)/pow(osize(j,2),2);
				fxx = 2*((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)))/pow(osize[i](j,0),2)*cos(yaw[i](j,0))+ 2*(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)))/pow(osize[i](j,1),2)*(-sin(yaw[i](j,0)));
				fyy = 2*((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)))/pow(osize[i](j,0),2)*sin(yaw[i](j,0))+ 2*(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)))/pow(osize[i](j,1),2)*(cos(yaw[i](j,0)));
				fzz = 2*((cz-oxyz[i](j,2)))/pow(osize[i](j,2),2);
				// fxx = 2*(cx-oxyz(j,0))/pow(osize(j,0),2);
				// fyy = 2*(cy-oxyz(j,1))/pow(osize(j,1),2);
				// fzz = 2*(cz-oxyz(j,2))/pow(osize(j,2),2);
				constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow + numHalfSpace * mpcWindow, numStates*i) = fxx;//x 
				constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow + numHalfSpace * mpcWindow, numStates*i+1) = fyy;//y 
				constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow + numHalfSpace * mpcWindow, numStates*i+2) = fzz;//z 	
				if (isDyamic[i][j]){
					constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow + numHalfSpace * mpcWindow, numStates * (mpcWindow + 1) + numControls*i + 3) = -1;
				}
				else{
					constraintMatrix.insert(i*numObs+j + (mpcWindow + 1) * numStates + numStates * (mpcWindow + 1) + numControls * mpcWindow + numHalfSpace * mpcWindow, numStates * (mpcWindow + 1) + numControls*i + 4) = -1;
				}
			}
		}
	}

	void mpcPlanner::castMPCToQPConstraintVectors(Eigen::Matrix<double,numStates,1> &xMax, Eigen::Matrix<double,numStates,1> &xMin,
		Eigen::Matrix<double,numControls,1> &uMax, Eigen::Matrix<double,numControls,1> &uMin,
		const Eigen::Matrix<double, numStates, 1>& x0, Eigen::Matrix<double, Eigen::Dynamic, 1> &lowerBound, Eigen::Matrix<double, Eigen::Dynamic, 1> &upperBound, 
		int numObs, int mpcWindow, 
		std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw){

		// evaluate the lower and the upper equality vectors
		int numHalfSpace = this->numHalfSpace_;
		Eigen::VectorXd lowerEquality = Eigen::MatrixXd::Zero(numStates * (mpcWindow + 1), 1);
		Eigen::VectorXd upperEquality;
		lowerEquality.block(0, 0, numStates, 1) = -x0;
		upperEquality = lowerEquality;
		lowerEquality = lowerEquality;

		Eigen::VectorXd lowerInequality
			= Eigen::MatrixXd::Zero(numStates * (mpcWindow + 1) + numControls * mpcWindow + numHalfSpace * mpcWindow, 1);
		Eigen::VectorXd upperInequality
			= Eigen::MatrixXd::Zero(numStates * (mpcWindow + 1) + numControls * mpcWindow + numHalfSpace * mpcWindow, 1);
		for (int i = 0; i < mpcWindow + 1; i++)
		{
			lowerInequality.block(numStates * i, 0, numStates, 1) = xMin;
			upperInequality.block(numStates * i, 0, numStates, 1) = xMax;
		}
		for (int i = 0; i < mpcWindow; i++)
		{
			lowerInequality.block(numControls * i + numStates * (mpcWindow + 1), 0, numControls, 1) = uMin;
			upperInequality.block(numControls * i + numStates * (mpcWindow + 1), 0, numControls, 1) = uMax;
		}
		if (this->numHalfSpace_){
			for (int i=0; i < mpcWindow; i++){
				// for (int j=0;j < numHalfSpace; j++){
					lowerInequality(numHalfSpace * i + 0 + numStates * (mpcWindow + 1) + numControls * mpcWindow) = -INFINITY;
					upperInequality(numHalfSpace * i + 0 + numStates * (mpcWindow + 1) + numControls * mpcWindow) = this->halfMax_(2);
					lowerInequality(numHalfSpace * i + 1 + numStates * (mpcWindow + 1) + numControls * mpcWindow) = this->halfMin_(2);
					upperInequality(numHalfSpace * i + 1 + numStates * (mpcWindow + 1) + numControls * mpcWindow) = INFINITY;
				// }
			}
		}


		Eigen::VectorXd lowerObstacle
			= Eigen::MatrixXd::Zero(numObs * mpcWindow, 1);
		Eigen::VectorXd upperObstacle
			= Eigen::MatrixXd::Ones(numObs * mpcWindow, 1)*INFINITY;
			// Zero(numObs * mpcWindow, 1);
		for (int i = 0; i < mpcWindow; i++){
			double cx, cy, cz;
			if (this->currentStatesSol_.size()!=0){
				cx = this->currentStatesSol_[i](0);
				cy = this->currentStatesSol_[i](1);
				cz = this->currentStatesSol_[i](2);
			}
			else{
				cx = this->currPos_(0);
				cy = this->currPos_(1);
				cz = this->currPos_(2);
			}
			for (int j = 0; j < numObs; j++){
				double fxyz,fxx,fyy,fzz;
				fxyz = pow((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)), 2)/pow(osize[i](j,0),2) + pow(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)), 2)/pow(osize[i](j,1),2) + pow((cz-oxyz[i](j,2)), 2)/pow(osize[i](j,2),2);
				fxx = 2*((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)))/pow(osize[i](j,0),2)*cos(yaw[i](j,0))+ 2*(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)))/pow(osize[i](j,1),2)*(-sin(yaw[i](j,0)));
				fyy = 2*((cx-oxyz[i](j,0))*cos(yaw[i](j,0))+(cy-oxyz[i](j,1))*sin(yaw[i](j,0)))/pow(osize[i](j,0),2)*sin(yaw[i](j,0))+ 2*(-(cx-oxyz[i](j,0))*sin(yaw[i](j,0))+(cy-oxyz[i](j,1))*cos(yaw[i](j,0)))/pow(osize[i](j,1),2)*(cos(yaw[i](j,0)));
				fzz = 2*((cz-oxyz[i](j,2)))/pow(osize[i](j,2),2);
				lowerObstacle(i*numObs+j) = 1 - fxyz + fxx * cx + fyy * cy + fzz * cz;
			}
		}
		
		lowerBound.resize(numStates * (mpcWindow+1) + numStates * (mpcWindow+1) + numControls * mpcWindow + numHalfSpace * mpcWindow + numObs * mpcWindow, 1);
		upperBound.resize(numStates * (mpcWindow+1) + numStates * (mpcWindow+1) + numControls * mpcWindow + numHalfSpace * mpcWindow + numObs * mpcWindow, 1);

		lowerBound << lowerEquality, lowerInequality, lowerObstacle;
		upperBound << upperEquality, upperInequality, upperObstacle;
	}

	void mpcPlanner::updateObstacleParam(const std::vector<staticObstacle> &staticObstacles, const std::vector<std::vector<Eigen::Vector3d>> &dynamicObstaclesPos, const std::vector<std::vector<Eigen::Vector3d>> &dynamicObstaclesSize, int &numObs, int mpcWindow, 
		std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &oxyz, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 3>> &osize, std::vector<Eigen::Matrix<double, Eigen::Dynamic, 1>> &yaw, 
		std::vector<std::vector<int>> &isDyamic){
		isDyamic.clear();
		isDyamic.resize(mpcWindow);
		numObs = staticObstacles.size()+dynamicObstaclesPos.size();
		int numDynamicOb = dynamicObstaclesPos.size();
		int numStaticOb = staticObstacles.size();
		oxyz.resize(mpcWindow);
		osize.resize(mpcWindow);
		yaw.resize(mpcWindow);
		for(int j=0; j<mpcWindow;j++){
			oxyz[j].resize(numObs,3);
			osize[j].resize(numObs,3);
			yaw[j].resize(numObs,1);
			isDyamic[j].resize(numObs);
			for(int i=0; i<numDynamicOb; i++){
				if (j<dynamicObstaclesPos[i].size()){
					oxyz[j](i,0) = dynamicObstaclesPos[i][j](0);
					oxyz[j](i,1) = dynamicObstaclesPos[i][j](1);
					oxyz[j](i,2) = dynamicObstaclesPos[i][j](2);
					osize[j](i,0) = dynamicObstaclesSize[i][j](0)/2 + this->dynamicSafetyDist_;
					osize[j](i,1) = dynamicObstaclesSize[i][j](1)/2 + this->dynamicSafetyDist_;
					osize[j](i,2) = dynamicObstaclesSize[i][j](2)/2 + this->dynamicSafetyDist_;
					yaw[j](i,0) = 0.0;
					isDyamic[j][i] = 1;
				}
				else{
					oxyz[j](i,0) = dynamicObstaclesPos[i].back()(0);
					oxyz[j](i,1) = dynamicObstaclesPos[i].back()(1);
					oxyz[j](i,2) = dynamicObstaclesPos[i].back()(2);
					osize[j](i,0) = dynamicObstaclesSize[i].back()(0)/2 + this->dynamicSafetyDist_;
					osize[j](i,1) = dynamicObstaclesSize[i].back()(1)/2 + this->dynamicSafetyDist_;
					osize[j](i,2) = dynamicObstaclesSize[i].back()(2)/2 + this->dynamicSafetyDist_;
					yaw[j](i,0) = 0.0;
					isDyamic[j][i] = 1;
				}
			}
			for(int i=0; i<numStaticOb; i++){
				oxyz[j](i+numDynamicOb,0) = staticObstacles[i].centroid(0);
				oxyz[j](i+numDynamicOb,1) = staticObstacles[i].centroid(1);
				oxyz[j](i+numDynamicOb,2) = staticObstacles[i].centroid(2);
				osize[j](i+numDynamicOb,0) = staticObstacles[i].size(0)/2 + this->staticSafetyDist_;
				osize[j](i+numDynamicOb,1) = staticObstacles[i].size(1)/2 + this->staticSafetyDist_;
				osize[j](i+numDynamicOb,2) = staticObstacles[i].size(2)/2 + this->staticSafetyDist_;
				yaw[j](i+numDynamicOb,0) = staticObstacles[i].yaw;
				isDyamic[j][i] = 0;
			}
		}
	}

	void mpcPlanner::getReferenceTraj(std::vector<Eigen::Vector3d>& referenceTraj){
		// find the nearest position in the reference trajectory
		double leastDist = std::numeric_limits<double>::max();
		double maxForwardTime = 3.0; // # 3.0s ahead
		int maxForwardIdx = maxForwardTime/this->ts_;
		int startIdx = this->lastRefStartIdx_;
		for (int i=this->lastRefStartIdx_; i<this->lastRefStartIdx_+maxForwardIdx; ++i){
			double dist = (this->currPos_ - this->inputTraj_[i]).norm();
			if (dist < leastDist){
				leastDist = dist;
				startIdx = i;
			}
		}
		this->lastRefStartIdx_ = startIdx; // update start idx
		referenceTraj.clear();
		for (int i=startIdx; i<startIdx+this->horizon_; ++i){
			if (i < int(this->inputTraj_.size())){
				referenceTraj.push_back(this->inputTraj_[i]);
			}
			else{
				referenceTraj.push_back(this->inputTraj_.back());
			}
		}
	}


	void mpcPlanner::getTrajectory(std::vector<Eigen::Vector3d>& traj){
		traj.clear();
		for (int i=0; i<this->currentStatesSol_.size(); ++i){
			// DVector states = this->currentStatesSol_.getVector(i);
			Eigen::VectorXd states = this->currentStatesSol_[i];
			Eigen::Vector3d p (states(0), states(1), states(2));
			traj.push_back(p);
		}
	}

	void mpcPlanner::getTrajectory(nav_msgs::Path& traj){
		traj.header.frame_id = "map";
		std::vector<Eigen::Vector3d> trajTemp;
		this->getTrajectory(trajTemp);
		for (int i=0; i<int(trajTemp.size()); ++i){
			geometry_msgs::PoseStamped ps;
			ps.pose.position.x = trajTemp[i](0);
			ps.pose.position.y = trajTemp[i](1);
			ps.pose.position.z = trajTemp[i](2);
			traj.poses.push_back(ps);
		}
	}

	Eigen::Vector3d mpcPlanner::getPos(double t){
		// int idx = int(t/this->ts_);
		// idx = std::max(0, std::min(idx, this->horizon_-1));
		// DVector states = this->currentStatesSol_.getVector(idx);
		// Eigen::Vector3d p (states(0), states(1), states(2));
		int idx = floor(t/this->ts_);
		double dt = t-idx*this->ts_;
		idx = std::max(0, std::min(idx, this->horizon_-1));
		Eigen::VectorXd states = this->currentStatesSol_[idx];
		Eigen::VectorXd nextStates = this->currentStatesSol_[std::min(idx+1, this->horizon_-1)];
		Eigen::Vector3d p;
		p(0) = states(0)+(nextStates(0)-states(0))/this->ts_*dt;
		p(1) = states(1)+(nextStates(1)-states(1))/this->ts_*dt;
		p(2) = states(2)+(nextStates(2)-states(2))/this->ts_*dt;
		return p;
	}

	Eigen::Vector3d mpcPlanner::getVel(double t){
		// int idx = int(t/this->ts_);
		// idx = std::max(0, std::min(idx, this->horizon_-1));
		// DVector states = this->currentStatesSol_.getVector(idx);
		// Eigen::Vector3d v (states(3), states(4), states(5));
		int idx = floor(t/this->ts_);
		double dt = t-idx*this->ts_;
		idx = std::max(0, std::min(idx, this->horizon_-1));
		Eigen::VectorXd states = this->currentStatesSol_[idx];
		Eigen::VectorXd nextStates = this->currentStatesSol_[std::min(idx+1, this->horizon_-1)];
		Eigen::Vector3d v;
		v(0) = states(3)+(nextStates(3)-states(3))/this->ts_*dt;
		v(1) = states(4)+(nextStates(4)-states(4))/this->ts_*dt;
		v(2) = states(5)+(nextStates(5)-states(5))/this->ts_*dt;
		return v;
	}

	Eigen::Vector3d mpcPlanner::getAcc(double t){
		// int idx = int(t/this->ts_);
		// idx = std::max(0, std::min(idx, this->horizon_-1));
		// DVector states = this->currentControlsSol_.getVector(idx);
		// Eigen::Vector3d a (states(0), states(1), states(2));
		int idx = floor(t/this->ts_);
		double dt = t-idx*this->ts_;
		idx = std::max(0, std::min(idx, this->horizon_-2));
		Eigen::VectorXd states = this->currentControlsSol_[idx];
		Eigen::VectorXd nextStates = this->currentControlsSol_[std::min(idx+1, this->horizon_-2)];
		Eigen::Vector3d a;
		a(0) = states(0)+(nextStates(0)-states(0))/this->ts_*dt;
		a(1) = states(1)+(nextStates(1)-states(1))/this->ts_*dt;
		a(2) = states(2)+(nextStates(2)-states(2))/this->ts_*dt;
		return a;
	}

	Eigen::Vector3d mpcPlanner::getRef(double t){
		int idx = floor(t/this->ts_);
		double dt = t-idx*this->ts_;
		idx = std::max(0, std::min(idx, this->horizon_-1));
		Eigen::MatrixXd states = this->ref_[idx];
		Eigen::MatrixXd nextStates = this->ref_[std::min(idx+1, this->horizon_-1)];
		Eigen::Vector3d r;
		r(0) = states(0,0)+(nextStates(0,0)-states(0,0))/this->ts_*dt;
		r(1) = states(1,0)+(nextStates(1,0)-states(1,0))/this->ts_*dt;
		r(2) = states(2,0)+(nextStates(2,0)-states(2,0))/this->ts_*dt;
		return r;
	}

	double mpcPlanner::getTs(){
		return this->ts_;
	}

	double mpcPlanner::getHorizon(){
		return this->horizon_;
	}

	void mpcPlanner::visCB(const ros::TimerEvent&){
		this->publishMPCTrajectory();
		this->publishCandidateTrajectory();
		this->publishHistoricTrajectory();
		this->publishLocalCloud();
		this->publishStaticObstacles();
		this->publishDynamicObstacles();
	}

	void mpcPlanner::publishMPCTrajectory(){
		if (not this->firstTime_){
			nav_msgs::Path traj;
			this->getTrajectory(traj);
			this->mpcTrajVisPub_.publish(traj);
		}
	}

	void mpcPlanner::publishCandidateTrajectory(){
		if (not this->firstTime_){
			if (this->candidateStates_.size() > 0){
				visualization_msgs::MarkerArray trajMsg;
				int countMarker = 0;
				for (int i=0; i<int(this->candidateStates_.size()); ++i){
					visualization_msgs::Marker text;
					text.header.frame_id = "map";
					text.header.stamp = ros::Time::now();
					text.ns = this->ns_;
					text.id = countMarker;
					text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
					text.lifetime = ros::Duration(0.1);
					// text.action = visualization_msgs::Marker::ADD;
					
					text.pose.position.x = this->candidateStates_[i].back()(0);
					text.pose.position.y = this->candidateStates_[i].back()(1);
					text.pose.position.z = this->candidateStates_[i].back()(2);
					
					text.scale.z = 0.5;  
					text.color.r = 1.0;
					text.color.g = 0.0;
					text.color.b = 1.0;
					text.color.a = 1.0;
					std::string message = "score: "+std::to_string(this->trajWeightedScore_[i]);
					// std::string message = "score: "+std::to_string(this->trajWeightedScore_[i]) + "consistent: " + std::to_string(this->trajScore_[i](0))
					// + "detour: " + std::to_string(this->trajScore_[i](1))+ "safty: " + std::to_string(this->trajScore_[i](2));
					text.text = message;

					trajMsg.markers.push_back(text);
					++countMarker;
					visualization_msgs::Marker traj;
					traj.header.frame_id = "map";
					traj.header.stamp = ros::Time::now();
					traj.ns = this->ns_;
					traj.id = countMarker;
					traj.type = visualization_msgs::Marker::LINE_STRIP;
					traj.scale.x = 0.1;
					traj.scale.y = 0.1;
					traj.scale.z = 0.1;
					traj.color.a = 0.4;
					traj.color.r = 1.0;
					traj.color.g = 0.0;
					traj.color.b = 1.0;
					traj.lifetime = ros::Duration(0.1);
					for (int j=0; j<int(this->candidateStates_[i].size());j++){
						geometry_msgs::Point p;
						Eigen::VectorXd state = this->candidateStates_[i][j];
						p.x = state(0); p.y = state(1); p.z = state(2);
						traj.points.push_back(p);
						++countMarker;
						trajMsg.markers.push_back(traj);
					}
				}
				this->candidateTrajPub_.publish(trajMsg);
			}
		}
	}

	void mpcPlanner::publishHistoricTrajectory(){
		if (not this->firstTime_){
			nav_msgs::Path histTraj;
			histTraj.header.frame_id = "map";
			for (int i=0; i<int(this->trajHist_.size()); ++i){
				geometry_msgs::PoseStamped ps;
				ps.pose.position.x = this->trajHist_[i](0);
				ps.pose.position.y = this->trajHist_[i](1);
				ps.pose.position.z = this->trajHist_[i](2);
				histTraj.poses.push_back(ps);
			}
			this->mpcTrajHistVisPub_.publish(histTraj);
		}
	}

	void mpcPlanner::publishLocalCloud(){
		if (this->currCloud_.size() != 0){
			pcl::PointXYZ pt;
			pcl::PointCloud<pcl::PointXYZ> cloud;

			for (int i=0; i<int(this->currCloud_.size()); ++i){
				pt.x = this->currCloud_[i](0);
				pt.y = this->currCloud_[i](1);
				pt.z = this->currCloud_[i](2);
				cloud.push_back(pt);
			}

			cloud.width = cloud.points.size();
			cloud.height = 1;
			cloud.is_dense = true;
			cloud.header.frame_id = "map";

			sensor_msgs::PointCloud2 cloudMsg;
			pcl::toROSMsg(cloud, cloudMsg);
			this->localCloudPub_.publish(cloudMsg);		
		}
	}

	void mpcPlanner::publishStaticObstacles(){
		if (this->refinedBBoxVertices_.size() != 0){
		    visualization_msgs::Marker line;
		    visualization_msgs::MarkerArray lines;

		    line.header.frame_id = "map";
		    line.type = visualization_msgs::Marker::LINE_LIST;
		    line.action = visualization_msgs::Marker::ADD;
		    line.ns = "mpc_static_obstacles";  
		    line.scale.x = 0.06;
		    line.color.r = 0;
		    line.color.g = 1;
		    line.color.b = 1;
		    line.color.a = 1.0;
		    line.lifetime = ros::Duration(0.2);
		    Eigen::Vector3d vertex_pose;
		    for(int i=0; i<int(this->refinedBBoxVertices_.size()); ++i){
		        bboxVertex v = this->refinedBBoxVertices_[i];
		        std::vector<geometry_msgs::Point> verts;
		        geometry_msgs::Point p;

				for (int j=0; j<int(v.vert.size());++j){
					p.x = v.vert[j](0); p.y = v.vert[j](1); p.z = v.vert[j](2);
		        	verts.push_back(p);
				}

		        int vert_idx[12][2] = {
		            {0,1},
		            {1,2},
		            {2,3},
		            {0,3},
		            {0,4},
		            {1,5},
		            {3,7},
		            {2,6},
		            {4,5},
		            {5,6},
		            {4,7},
		            {6,7}
		        };
		        for (int j=0;j<12;++j){
		            line.points.push_back(verts[vert_idx[j][0]]);
		            line.points.push_back(verts[vert_idx[j][1]]);
		        }
		        lines.markers.push_back(line);
		        line.id++;
		    }
		    this->staticObstacleVisPub_.publish(lines);		


		    // facing direction	
		    visualization_msgs::Marker currPoseMarker;
	    	currPoseMarker.header.frame_id = "map";
			currPoseMarker.header.stamp = ros::Time();
			currPoseMarker.ns = "currPoseVis";
			currPoseMarker.id = 0;
			currPoseMarker.type = visualization_msgs::Marker::ARROW;
			currPoseMarker.action = visualization_msgs::Marker::ADD;
			currPoseMarker.pose.position.x = this->currPos_(0);
			currPoseMarker.pose.position.y = this->currPos_(1);
			currPoseMarker.pose.position.z = this->currPos_(2);
			// double angle = atan2(this->currVel_(1), this->currVel_(0));
			currPoseMarker.pose.orientation = trajPlanner::quaternion_from_rpy(0, 0, this->angle_);
			currPoseMarker.lifetime = ros::Duration(0.2);
			currPoseMarker.scale.x = 0.4;
			currPoseMarker.scale.y = 0.2;
			currPoseMarker.scale.z = 0.2;
			currPoseMarker.color.a = 1.0;
			currPoseMarker.color.r = 0.0;
			currPoseMarker.color.g = 0.5;
			currPoseMarker.color.b = 1.0;
			this->facingPub_.publish(currPoseMarker);
		}
	}

	void mpcPlanner::publishDynamicObstacles(){
		if (this->dynamicObstaclesPos_.size() != 0){
		    visualization_msgs::MarkerArray lines;
			int obIdx = 0;
			for (int i = 0; i<this->dynamicObstaclesPos_.size();i++){
				visualization_msgs::Marker line;
				line.header.frame_id = "map";
				line.type = visualization_msgs::Marker::LINE_LIST;
				line.action = visualization_msgs::Marker::ADD;
				line.ns = "mpc_dynamic_obstacles";  
				line.scale.x = 0.06;
				line.lifetime = ros::Duration(0.1);
				line.id = obIdx;
				if (obIdx == this->obIdx_){
					line.color.r = 1;
					line.color.g = 0;
					line.color.b = 0;
					line.color.a = 1.0;
				}
				else if (obIdx != this->obIdx_){
					line.color.r = 0;
					line.color.g = 0;
					line.color.b = 1;
					line.color.a = 1.0;
				}
				
				double x = this->dynamicObstaclesPos_[i][0](0); 
				double y = this->dynamicObstaclesPos_[i][0](1); 
				double z = this->dynamicObstaclesPos_[i][0](2);
				double x_width = this->dynamicObstaclesSize_[i][0](0);
				double y_width = this->dynamicObstaclesSize_[i][0](1);
				double z_width = this->dynamicObstaclesSize_[i][0](2);
				
				std::vector<geometry_msgs::Point> verts;
				geometry_msgs::Point p;
				// vertice 0
				p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
				verts.push_back(p);

				// vertice 1
				p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
				verts.push_back(p);

				// vertice 2
				p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z-z_width / 2.;
				verts.push_back(p);

				// vertice 3
				p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z-z_width / 2.;
				verts.push_back(p);

				// vertice 4
				p.x = x-x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
				verts.push_back(p);

				// vertice 5
				p.x = x-x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
				verts.push_back(p);

				// vertice 6
				p.x = x+x_width / 2.; p.y = y+y_width / 2.; p.z = z+z_width / 2.;
				verts.push_back(p);

				// vertice 7
				p.x = x+x_width / 2.; p.y = y-y_width / 2.; p.z = z+z_width / 2.;
				verts.push_back(p);
				
				int vert_idx[12][2] = {
					{0,1},
					{1,2},
					{2,3},
					{0,3},
					{0,4},
					{1,5},
					{3,7},
					{2,6},
					{4,5},
					{5,6},
					{4,7},
					{6,7}
				};
				
				for (size_t i=0;i<12;i++){
					line.points.push_back(verts[vert_idx[i][0]]);
					line.points.push_back(verts[vert_idx[i][1]]);
				}
				
				lines.markers.push_back(line);
				
				obIdx++;
			}
		    this->dynamicObstacleVisPub_.publish(lines);	
		}	
	}
}