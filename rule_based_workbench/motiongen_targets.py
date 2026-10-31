"""Rule task poses only: no IK, joint search, trajectory generation or acceptance."""
from dataclasses import asdict
from itertools import product
import math
from types import SimpleNamespace


REFERENCE = dict(
    source='curobo-playground-20260928-140507/matched-native-full/initial-plan.json',
    cup_profile_sha256='c10640f5aa674ab94ab84fe3433a3865ef8fe1aec9d72ceda0a0d00d1dd9002f',
    urdf_sha256='7bf205b7252905c55a4857a749f727756bf00be736811c047cdf8b8fd70759ff',
    native_geometry_sha256='6f76292057aebc8aa6ace26b651060055386de72208a7deb4ffa4e4d7f188ab0',
    envelope_sha256='7ef36c7e8769647ccd13c4aa4ed6c08a8a4c5757bcc188022ed8616c0a438e7c',
    grasp_offset_m=.010, pitch_rad=.6370061007603225, roll_rad=.000624422993328988,
    cup_from_tool_position_m=[-.127462024215198, 0., .13157404134421807])

PLACEMENT_INWARD_BIAS_M=.001  # User20261009: preserve 2mm tray-edge acceptance.


def _pose(value):
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    return value if isinstance(value, RigidTransform) else RigidTransform(**value)


def _hand_points_tool(planner, jaw_position):
    """Use real pad/metal vertices and URDF FK; never solve arm joint targets."""
    if not math.isfinite(jaw_position) or not 0 <= jaw_position <= planner.open_position_m:
        raise ValueError('finite jaw position within exported opening required')
    state = dict(planner.home_state, left_joint7=jaw_position, left_joint8=jaw_position)
    frames = planner.fk.forward(state)
    tool_from_robot = planner.fk.pose(frames['left_link6']).inverse()
    relative = {name: tool_from_robot.compose(planner.fk.pose(frames[name]))
                for name in ('left_link6', 'left_link7', 'left_link8')}
    return [(part.name, [relative[part.link].transform_point(v) for v in part.vertices])
            for part in planner.parts if part.link in ('left_link6', 'left_link7', 'left_link8')]


def checked_reference(planner,data,*,reference=None,urdf_path=None):
    ref = REFERENCE if reference is None else reference
    expected = dict(cup_profile_sha256=planner.cup.profile_sha256,
                    urdf_sha256=data['urdf_sha256'],
                    native_geometry_sha256=planner.identity['native_geometry_sha256'],
                    envelope_sha256=planner.identity['envelope_sha256'])
    if reference is None and urdf_path is not None and ref['urdf_sha256']!=expected['urdf_sha256']:
        from a1_asset_derivation import derive_cup_reference
        ref=derive_cup_reference(ref,urdf_path)
    assisted=getattr(planner,'operator_assisted_cup_geometry',None)
    selection=getattr(planner,'scene_asset_set',None)
    if reference is None and selection is not None:
        from scene_assets import validate
        validate(selection)
        if (selection['cup']['interaction_profile_sha256']!=expected['cup_profile_sha256']
                or any(ref.get(key)!=expected[key] for key in expected if key!='cup_profile_sha256')):
            raise ValueError('selected cup seed requires the same checked A1 mechanical geometry')
        ref=dict(ref,cup_profile_sha256=expected['cup_profile_sha256'],
            asset_derivation=dict(scope='unverified_task_pose_seed',asset_set_digest=selection['asset_set_digest'],
                parent_cup_profile_sha256=ref['cup_profile_sha256'],current_cup_profile_sha256=expected['cup_profile_sha256'],
                contact_and_path_acceptance_reused=False))
    if reference is None and assisted is not None:
        # This retains a numerical search seed, never the previous cup's grasp proof.
        if (assisted.get('profile_sha256')!=expected['cup_profile_sha256']
                or any(ref.get(key)!=expected[key] for key in expected if key!='cup_profile_sha256')):
            raise ValueError('assisted cup seed requires the same checked A1 mechanical geometry')
        ref=dict(ref,cup_profile_sha256=expected['cup_profile_sha256'],
            assistance_derivation=dict(scope='unverified_task_pose_seed',
                parent_cup_profile_sha256=ref['cup_profile_sha256'],
                current_cup_profile_sha256=expected['cup_profile_sha256'],
                contact_and_path_acceptance_reused=False))
    if not ref.get('source') or any(ref.get(key) != value for key, value in expected.items()):
        raise ValueError('cup-relative reference does not match current cup/A1-cal geometry')
    return ref,expected


def reference_grasp(planner,data,*,reference=None,urdf_path=None):
    """Retain the reviewed depth seed; it is not a new pose/contact certificate."""
    ref,expected=checked_reference(planner,data,reference=reference,urdf_path=urdf_path)
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    numbers = [ref['grasp_offset_m'], ref['pitch_rad'], ref['roll_rad'],
               *ref['cup_from_tool_position_m'], planner.GRASP_CENTER_OFFSET_M]
    if not all(math.isfinite(v) for v in numbers):
        raise ValueError('finite relative reference and current grasp offset required')
    pitch, roll = ref['pitch_rad'], ref['roll_rad']
    # R_y(pitch) R_x(roll), equivalent to native Rotation.from_euler('xyz',[roll,pitch,0]).
    cp, sp, cr, sr = math.cos(pitch/2), math.sin(pitch/2), math.cos(roll/2), math.sin(roll/2)
    delta = planner.GRASP_CENTER_OFFSET_M-ref['grasp_offset_m']
    position = tuple(v+(delta if index == 0 else 0.) for index, v in enumerate(ref['cup_from_tool_position_m']))
    cup_from_tool = RigidTransform(position, (cp*cr, cp*sr, sp*cr, -sp*sr))
    return ref,expected,cup_from_tool


def reserved_grasp_candidates(planner,observation,data,ref,seed,*,closed_jaw_m):
    """Screen a bounded native task-pose domain, keeping depth and yaw fixed."""
    from grasp_contact_geometry import analyze_contact_pose
    from grasp_contact_certificate import GRASP_CONTACT_POLICY
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    if abs(planner.GRASP_CENTER_OFFSET_M-GRASP_CONTACT_POLICY['grasp_depth_m'])>1e-12:
        raise ValueError('reserved grasp policy requires the reviewed 0.016 m depth')
    lo,hi=planner.cup.candidate_grasp_band_m
    height_bounds=(lo+.015,hi+.075)  # Existing native bounded terminal domain.
    heights=sorted(set([seed.position[2],*(height_bounds[0]+i*(height_bounds[1]-height_bounds[0])/16 for i in range(17))]))
    orientations=[(ref['pitch_rad'],ref['roll_rad']),(.35,0.),(.55,0.),(.1,0.),(.8,0.)]
    cup=observation.sample.world_from_cup
    normal=cup.inverse().rotate(planner.table.world_from_shape.rotate((0.,0.,1.)))
    accepted=[];rejected=[];total_accepted=0
    for pitch,roll in dict.fromkeys(orientations):
        group=[]
        if not .1<=pitch<=.8 or not -.3<=roll<=.3:raise ValueError('reference outside native terminal orientation domain')
        cp,sp,cr,sr=math.cos(pitch/2),math.sin(pitch/2),math.cos(roll/2),math.sin(roll/2)
        for height in heights:
            if not height_bounds[0]<=height<=height_bounds[1]:continue
            pose=RigidTransform((seed.position[0],seed.position[1],height),(cp*cr,cp*sr,sp*cr,-sp*sr))
            evidence=analyze_contact_pose(planner,data,pose,closed_jaw_m=closed_jaw_m,table_normal_cup=normal)
            if evidence.get('passed') is not True:
                rejected.append(dict(pitch_rad=pitch,roll_rad=roll,height_m=height,reason=evidence.get('reason'),
                    contact_height_interval_m=evidence.get('contact_height_interval_m')))
                continue
            lower,upper=evidence['contact_height_interval_m']
            margin=min(lower-.035,.1-upper)
            group.append(dict(cup_from_tool=asdict(pose),pitch_rad=pitch,roll_rad=roll,
                minimum_contact_height_margin_m=margin,contact_region=evidence))
        group.sort(key=lambda row:(-row['minimum_contact_height_margin_m'],abs(row['cup_from_tool']['position'][2]-seed.position[2])))
        total_accepted+=len(group);accepted.extend(group[:4])
    return accepted,dict(method='bounded native task-space height lattice; up to four heights per orientation',
        depth_m=planner.GRASP_CENTER_OFFSET_M,yaw_change_rad=0.,pitch_bounds_rad=[.1,.8],roll_bounds_rad=[-.3,.3],
        height_bounds_m=list(height_bounds),height_intervals=16,maximum_returned_candidates=20,
        rejected=rejected,accepted_count=total_accepted,global_infeasibility_claim=False)


def placement_support_profile(cup):
    """Native placement input only: bottom footprint and mouth height, no mesh.

    Native placement_candidates reads the first band's radius as a footprint.
    A frustum's radius_m is its maximum radius, so pass a separate support view;
    collision/contact consumers must continue to receive the original cup.
    """
    bottom=min(cup.bands,key=lambda band:band.z_min_m)
    return SimpleNamespace(bands=(SimpleNamespace(z_min_m=bottom.z_min_m,
        z_max_m=max(band.z_max_m for band in cup.bands),radius_m=cup.bottom_radius_m),))


def supported_cup_pose(scene,cup,orientation,xy):
    """Keep cup orientation/support XY; bring its true lowest point to the plane."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.tasks.manager_based.coffee_make.interaction_evaluation import InteractionEvaluator
    bottom=min(cup.bands,key=lambda band:band.z_min_m)
    support=scene.world_from_support
    pose=RigidTransform(support.transform_point((*xy,-bottom.z_min_m)),orientation)
    height=-InteractionEvaluator._lowest_z(SimpleNamespace(cup=cup),support.inverse().compose(pose))
    if abs(height)<=1e-12:return pose,0.
    normal=support.rotate((0.,0.,1.))
    return RigidTransform(tuple(p+height*n for p,n in zip(pose.position,normal)),orientation),height


def generate_targets(planner, observation, data, *, reference=None,urdf_path=None,closed_jaw_m=None):
    """Bind reserve-screened cup-relative grasps and support-local cup goals."""
    ref,expected,seed=reference_grasp(planner,data,reference=reference,urdf_path=urdf_path)
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    from robot_sim.tasks.manager_based.coffee_make.relative_targets import (
        placement_candidates, eroded_support_polygon, _nearest_polygon_point)
    from robot_sim.tasks.manager_based.coffee_make.outlet_alignment import OUTLET_TOLERANCE_M, outlet_distance
    from grasp_contact_certificate import GRASP_CONTACT_POLICY
    # The existing reviewed 83/65 mm A1 metal definitions differ by 18 mm.
    closed_jaw_m=planner.open_position_m-.009 if closed_jaw_m is None else closed_jaw_m
    candidates,selection=reserved_grasp_candidates(planner,observation,data,ref,seed,closed_jaw_m=closed_jaw_m)
    cup = observation.sample.world_from_cup
    hand = _hand_points_tool(planner, planner.open_position_m)
    if not hand:
        raise ValueError('complete active hand geometry required')
    extent = max(point[0] for _, points in hand for point in points)
    grasps=[]
    for index,candidate in enumerate(candidates):
        pose=RigidTransform(**candidate['cup_from_tool']);tool=cup.compose(pose)
        pre=tool.compose(RigidTransform((-(extent+max(b.radius_m for b in planner.cup.bands)),0.,0.)))
        grasps.append(dict(candidate,candidate_id=f'contact-reserve-{index:02d}',source=ref['source'],
            world_from_tool=asdict(tool),world_from_pregrasp=asdict(pre),tool_from_cup=asdict(pose.inverse()),
            original_grasp_offset_m=ref['grasp_offset_m'],current_grasp_offset_m=planner.GRASP_CENTER_OFFSET_M,
            cup_x_delta_m=planner.GRASP_CENTER_OFFSET_M-ref['grasp_offset_m'],yaw_change_rad=0.,
            acceptance='continuous potential contact reserve only; native closing and GPU path checks still required'))
    scene = observation.scene
    acceptance = data['native_identity']['acceptance']['edge_margin_m']
    margin = acceptance+max(planner.PLACEMENT_TRACKING_RESERVE_M,planner.MACHINE_PLACEMENT_RESERVE_M)
    polygon = eroded_support_polygon(scene, planner.cup.bottom_radius_m, margin)
    heading = scene.world_from_support.inverse().rotate(cup.rotate((1., 0., 0.)))
    yaw = math.atan2(heading[1], heading[0])
    placements, rejected = [], []
    tracking_reserve=planner.PLACEMENT_TRACKING_RESERVE_M-PLACEMENT_INWARD_BIAS_M
    if tracking_reserve<=0:raise ValueError('inward bias must preserve a positive receiving reserve')
    target_tolerance=OUTLET_TOLERANCE_M-tracking_reserve
    if target_tolerance<=0:raise ValueError('placement tracking reserve exceeds outlet tolerance')

    def append(xy, source, preferred=False):
        nominal_xy = tuple(xy)
        nominal_pose,_=supported_cup_pose(scene,planner.cup,cup.orientation_wxyz,nominal_xy)
        nominal_distance=outlet_distance(scene,planner.cup,nominal_pose)
        # Tray-local +Y goes inward; neither grasp depth nor cup height changes.
        xy = (nominal_xy[0],nominal_xy[1]+PLACEMENT_INWARD_BIAS_M)
        if not polygon or math.dist(_nearest_polygon_point(xy, polygon), xy) > 1e-10:
            rejected.append(dict(source=source, support_xy_m=xy, reason='outside current eroded support polygon'))
            return
        if any(math.dist(row['support_xy_m'], xy) < 1e-10 for row in placements):
            return
        pose,support_height=supported_cup_pose(scene,planner.cup,cup.orientation_wxyz,xy)
        distance=outlet_distance(scene,planner.cup,pose)
        if distance>target_tolerance+1e-12:
            rejected.append(dict(source=source,support_xy_m=xy,reason='outside reserved receiving goal',outlet_distance_m=distance))
            return
        placements.append(dict(candidate_id=f'placement-{len(placements):02d}', source=source, preferred=preferred,
                               support_xy_m=xy, world_from_cup=asdict(pose),
                               tool_pose_binding='selected grasp and actual native closing relation',
                               planning_edge_margin_m=margin,outlet_distance_m=distance,accepted_trajectory=False,
                               nominal_support_xy_m=nominal_xy,nominal_outlet_distance_m=nominal_distance,
                               inward_bias_m=PLACEMENT_INWARD_BIAS_M,
                               **({'support_normal_adjustment_m':support_height,
                                   'support_contact_geometry':'native lowest point; dynamic stability unverified'}
                                  if support_height else {})))

    for candidate in placement_candidates(scene, placement_support_profile(planner.cup), seed.inverse(),
            edge_margin_m=margin,cup_yaw_rad=yaw,outlet_tolerance_m=target_tolerance):
        append(candidate.support_xy_m, 'native-outlet-aligned-support-geometry')
    placements.sort(key=lambda row:row['nominal_outlet_distance_m'])
    if placements:placements[0]['preferred']=True
    return dict(schema_version='motiongen_rule_targets.v1', task_space_only=True,
                reference_identity=expected,grasp_contact_policy=dict(GRASP_CONTACT_POLICY),
                grasp_candidate_selection=selection,grasp_candidates=grasps,placement_candidates=placements,
                **({'reference_derivation':ref['calibration_derivation']} if 'calibration_derivation' in ref else {}),
                **({'assisted_reference_derivation':ref['assistance_derivation']} if 'assistance_derivation' in ref else {}),
                placement_goal=dict(schema_version='cup_outlet_goal.v1',metric='cup_mouth_world_xy_to_liquid_outlet',
                    tolerance_m=OUTLET_TOLERANCE_M,target_tolerance_m=target_tolerance,
                    tracking_reserve_m=tracking_reserve,
                    nominal_tracking_reserve_m=planner.PLACEMENT_TRACKING_RESERVE_M,
                    inward_bias_m=PLACEMENT_INWARD_BIAS_M),
                rejected_placement_targets=rejected,
                placement_depth_policy='cup support goal independent of grasp depth; +1mm tray-local inward bias; legacy -0.0075 is not applied',
                placement_selection_policy='nearest nominal receiving goal first; biased candidates satisfy receiving and full-footprint margins',
                no_horizontal_transport_twist=True)


def straight_waypoints(start, goal, *, linear_axis=1, spacing_m=.0015, project_to_goal_frame=True):
    """Short collinear goals for the native axis-aligned partial pose metric."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    if type(linear_axis) is not int or not 0<=linear_axis<3 or not math.isfinite(spacing_m) or spacing_m<=0:
        raise ValueError('valid linear axis and positive spacing required')
    local=(goal.inverse().compose(start).position if project_to_goal_frame else
           tuple(a-b for a,b in zip(start.position,goal.position)))
    transverse=math.sqrt(sum(v*v for i,v in enumerate(local) if i!=linear_axis))
    count=max(1,math.ceil(transverse/spacing_m))
    return [RigidTransform(tuple(a+(b-a)*i/count for a,b in zip(start.position,goal.position)),goal.orientation_wxyz)
            for i in range(1,count+1)]


def carry_targets(planner, observation, grasp, placement, *, jaw_position_m=None, travel_lift_m=.02):
    """Explicit rule corners; MotionGen owns all IK and paths between these poses."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    if not math.isfinite(travel_lift_m) or travel_lift_m < 0:
        raise ValueError('finite nonnegative travel lift required')
    cup, goal = observation.sample.world_from_cup, _pose(placement['world_from_cup'])
    if min(math.dist(goal.orientation_wxyz, cup.orientation_wxyz),
           math.dist(goal.orientation_wxyz, tuple(-v for v in cup.orientation_wxyz))) > 1e-7:
        raise ValueError('carry targets may not introduce a cup yaw/tilt change')
    cup_from_tool = _pose(grasp['cup_from_tool'])
    hand = _hand_points_tool(planner, planner.open_position_m if jaw_position_m is None else jaw_position_m)
    components = [(name, [cup_from_tool.transform_point(v) for v in points]) for name, points in hand]
    for band in planner.cup.bands:
        components.append(('cup/'+band.shape_path,
            [(sx*band.radius_m, sy*band.radius_m, z) for sx, sy in product((-1, 1), repeat=2)
             for z in (band.z_min_m, band.z_max_m)]))
    outward = observation.scene.world_from_support.rotate((0., -1., 0.))
    length = math.hypot(*outward[:2])
    if length < 1e-6:
        raise ValueError('machine front has no horizontal direction')
    outward = (outward[0]/length, outward[1]/length, 0.)
    projection = lambda point: sum(a*b for a, b in zip(point, outward))
    lifted = RigidTransform((cup.position[0], cup.position[1], cup.position[2]+.02), cup.orientation_wxyz)
    hover = RigidTransform((goal.position[0], goal.position[1], goal.position[2]+.005), cup.orientation_wxyz)
    z = max(lifted.position[2], hover.position[2])+travel_lift_m
    from motiongen_motion import TASK_POSITION_TOLERANCE_M,TASK_ORIENTATION_TOLERANCE_RAD
    reach=max(math.dist(point,cup_from_tool.position) for _,points in components for point in points)
    solver_reserve=TASK_POSITION_TOLERANCE_M+2*reach*math.sin(TASK_ORIENTATION_TOLERANCE_RAD/2)
    lane_clearance=.017+solver_reserve
    from robot_sim.tasks.manager_based.coffee_make.outlet_alignment import outlet_position
    from cup_contact import ALLOWANCE_M
    cup_top=max(cup.rotate(point)[2] for name,points in components if name.startswith('cup/') for point in points)
    under_outlet=outlet_position(observation.scene)[2]-cup_top-solver_reserve
    if under_outlet<max(lifted.position[2],hover.position[2]):
        raise ValueError('no transverse height above lift/hover and below outlet')
    z=min(z,under_outlet)
    raised = RigidTransform((cup.position[0], cup.position[1], z), cup.orientation_wxyz)
    lane, pair_bounds = projection(raised.position), []
    rotated_components = [(name, [cup.rotate(point) for point in points]) for name, points in components]
    for shape in observation.scene.shapes:
        corners = [shape.world_from_shape.transform_point(tuple(sign*d/2 for sign, d in zip(signs, shape.dimensions_m)))
                   for signs in product((-1, 1), repeat=3)]
        front = max(map(projection, corners))
        low, high = min(p[2] for p in corners), max(p[2] for p in corners)
        for name, offsets in rotated_components:
            required=max(0.,solver_reserve-ALLOWANCE_M) if name.startswith('cup/') else lane_clearance
            # The continuous cross also descends to the insertion height.
            vertical = max(min(z,hover.position[2])+min(p[2] for p in offsets)-high,
                           low-max(z,hover.position[2])-max(p[2] for p in offsets), 0.)
            nearest = min(map(projection, offsets))
            if vertical < required:
                lane = max(lane, front+required-nearest)
            pair_bounds.append((shape.shape_path, name, vertical, nearest, front))

    # Leaving the tray is a task requirement even when the raised cup is above
    # its collision volume. Use original geometry, never the inset cup proxy.
    tray_front = max(map(projection, observation.scene.support_polygon_world_m))
    cup_nearest = min(projection(point) for name, points in rotated_components
                      if name.startswith('cup/') for point in points)
    lane = max(lane, tray_front + solver_reserve - cup_nearest)

    def on_lane(position):
        shift = lane-projection(position)
        return RigidTransform(tuple(p+shift*axis for p, axis in zip(position, outward)), cup.orientation_wxyz)

    rear = on_lane(raised.position)
    front = on_lane((goal.position[0], goal.position[1], z))
    lowered_front = RigidTransform((front.position[0], front.position[1], hover.position[2]), cup.orientation_wxyz)
    stages = [('lift_withdraw',rear),('cross',lowered_front),('insert',hover),('lower',goal)]
    clearance = min((max(vertical, lane+nearest-edge) for _,name,vertical,nearest,edge in pair_bounds
                     if not name.startswith('cup/')),default=math.inf)
    return dict(recipe='normal-v31-dynamic-targets',
                nominal_stage_duration_s=dict(lift_withdraw=4.3,cross=170/30,insert=3.,lower=29/30),
                stages=[dict(name=name, world_from_cup=asdict(pose),
                            world_from_tool=asdict(pose.compose(cup_from_tool)), support_contact=name == 'lower')
                        for name, pose in stages], required_transport_clearance_m=.017,
                base_clearance_m=.015, tracking_reserve_m=.002,
                solver_pose_reserve_m=solver_reserve,planned_lane_clearance_m=lane_clearance,
                cup_environment_allowance_m=ALLOWANCE_M,cross_cup_bottom_world_z_m=z,
                cross_cup_bottom_height_range_m=[min(z,hover.position[2]),max(z,hover.position[2])],
                cross_cup_top_clearance_m=outlet_position(observation.scene)[2]-z-cup_top,
                conservative_cross_clearance_m=clearance if math.isfinite(clearance) else None,
                outward_world=outward, lane_projection_m=lane, yaw_change_rad=0.,
                withdrawal_distance_m=projection(rear.position)-projection(raised.position),
                tray_exit=dict(required=True, original_geometry=True,
                    tray_front_projection_m=tray_front,
                    nearest_cup_projection_m=lane+cup_nearest,
                    clearance_m=lane+cup_nearest-tray_front,
                    solver_reserve_m=solver_reserve,
                    metric='entire original cup horizontal projection beyond tray front before cross'),
                hand_envelope_jaw_position_m=planner.open_position_m if jaw_position_m is None else jaw_position_m,
                clearance_scope='hand17mm; cup existing3mm allowance; mouth below outlet; full robot/path audit remains required')

def retreat_target(planner, placed_cup, placed_tool, *, clearance_m=.017, lateral_world=None):
    """Withdraw until the original open hand clears the original cup in XY."""
    from robot_sim.asset_cfg.objects.coffee_machine import RigidTransform
    direction=placed_tool.rotate((-1.,0.,0.))
    if lateral_world is not None:
        if (len(lateral_world)!=3 or not all(math.isfinite(v) for v in lateral_world)
                or abs(lateral_world[2])>1e-9 or math.hypot(*lateral_world[:2])<1e-6):
            raise ValueError('retreat lateral direction must be finite, horizontal and nonzero')
        norm=math.hypot(*lateral_world[:2])
        direction=tuple(a+b/norm for a,b in zip(direction,lateral_world))
        norm=math.sqrt(sum(v*v for v in direction));direction=tuple(v/norm for v in direction)
    horizontal=math.hypot(direction[0],direction[1])
    if horizontal<1e-6 or not math.isfinite(clearance_m) or clearance_m<0:
        raise ValueError('retreat requires a horizontal component and finite clearance')
    axis=(direction[0]/horizontal,direction[1]/horizontal,0.)
    dot=lambda p:sum(a*b for a,b in zip(axis,p))
    hand=_hand_points_tool(planner,planner.open_position_m)
    nearest=min(dot(placed_tool.transform_point(p)) for _,points in hand for p in points)
    radial=math.hypot(dot(placed_cup.rotate((1.,0.,0.))),dot(placed_cup.rotate((0.,1.,0.))))
    vertical=dot(placed_cup.rotate((0.,0.,1.)))
    edge=max(dot(placed_cup.position)+max(b.z_min_m*vertical,b.z_max_m*vertical)+b.radius_m*radial
             for b in planner.cup.bands)
    distance=max(0.,(edge+clearance_m-nearest)/horizontal)
    pose=RigidTransform(tuple(p+distance*v for p,v in zip(placed_tool.position,direction)),placed_tool.orientation_wxyz)
    return dict(world_from_tool=asdict(pose),withdraw_distance_m=distance,
        hand_cup_clearance_m=nearest+distance*horizontal-edge,required_clearance_m=clearance_m,
        direction_world=direction,
        metric='horizontal projection of original open hand and original cup volume',
        collision_proxy_used=False)
