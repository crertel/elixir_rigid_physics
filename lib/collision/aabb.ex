defmodule ElixirRigidPhysics.Collision.AABB do
  require Record
  Record.defrecord(:aabb, min: {0.0, 0.0, 0.0}, max: {0.0, 0.0, 0.0})

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
  require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
  require ElixirRigidPhysics.Geometry.Box, as: Box

  alias Graphmath.Quatern

  @doc """
  Creates a world-space AABB given a body record.

  ## Examples

    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> s = Sphere.sphere(radius: 3.0)
    iex> b = Body.create(s, position: {2,1,2}, orientation: {1.0, 0.0, 0.0, 0.0})
    iex> AABB.create_world_from_body(b)
    {:aabb, {-1.0, -2.0, -1.0,},{5.0, 4.0, 5.0}}

    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> s = Capsule.capsule(axial_length: 4.0, cap_radius: 1.0)
    iex> b = Body.create(s, position: {3.0, 4.0, 5.0}, orientation: {1.0, 0.0, 0.0, 0.0})
    iex> AABB.create_world_from_body(b)
    {:aabb, {2.0, 1.0, 4.0}, {4.0, 7.0, 6.0}}

    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> s = Capsule.capsule(axial_length: 4.0, cap_radius: 1.0)
    iex> b = Body.create(s, position: {0.0, 0.0, 0.0}, orientation: {1, 0.0, 0.0, 0.0})
    iex> AABB.create_world_from_body(b)
    {:aabb, {-1.0, -3.0, -1.0}, {1.0, 3.0, 1.0} }

    iex> sqrt_half = :math.sqrt(0.5)
    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> s = Capsule.capsule(axial_length: 4.0, cap_radius: 1.0)
    iex> b = Body.create(s, position: {0.0, 0.0, 0.0}, orientation: {sqrt_half, sqrt_half, 0.0, 0.0})
    iex> {:aabb, min, max} = AABB.create_world_from_body(b)
    iex> Graphmath.Vec3.equal(min, {-1.0, -1.0, -3.0}, 0.000001 ) and Graphmath.Vec3.equal(max, {1.0, 1.0, 3.0}, 0.000001 )
    true

    iex> sqrt_half = :math.sqrt(0.5)
    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> s = Capsule.capsule(axial_length: 4.0, cap_radius: 1.0)
    iex> b = Body.create(s, position: {1.0, 2.0, 3.0}, orientation: {sqrt_half, sqrt_half, 0.0, 0.0})
    iex> {:aabb, min, max} = AABB.create_world_from_body(b)
    iex> Graphmath.Vec3.equal(min, {0.0, 1.0, 0.0}, 0.000001 ) and Graphmath.Vec3.equal(max, {2.0, 3.0, 6.0}, 0.000001 )
    true

    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Box, as: Box
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> s = Box.box(width: 1.0, height: 2.0, depth: 3.0)
    iex> b = Body.create(s, position: {0.0, 0.0, 0.0}, orientation: Graphmath.Quatern.from_axis_angle(:math.pi()/4, {1.0,0.0,0.0}))
    iex> {:aabb, min, max} = AABB.create_world_from_body(b)
    iex> newmin = {-0.5, -1.76777, -1.76777}
    iex> newmax = {0.5, 1.76777, 1.76777}
    iex> Graphmath.Vec3.equal(min, newmin, 0.0001 ) and Graphmath.Vec3.equal(max, newmax, 0.00001 )
    true
  """
  def create_world_from_body(
        Body.body(shape: Sphere.sphere(radius: r) = _sphere, position: {px, py, pz})
      ) do
    aabb(
      min: {px - r, py - r, pz - r},
      max: {px + r, py + r, pz + r}
    )
  end

  @near_infinite 1.0e280
  def create_world_from_body(
        Body.body(shape: shape, position: {px, py, pz}, orientation: orientation)
      ) do
    # get local-space AABB
    aabb(min: {minx, miny, minz} = min , max: {maxx, maxy, maxz} = max) = create_local_from_shape(shape)

    # recreate all corners of AABB, using naming from octants ( https://en.wikipedia.org/wiki/Octant_(solid_geometry) )
    c1 = max # +x, +y, +z
    c2 = {minx, maxy, maxz} # -x, +y, +z
    c3 = {minx, miny, maxz} # -x, -y, +z
    c4 = {maxx, miny, maxz} # +x, -y, +z
    c5 = {maxx, maxy, minz} # +x, +y, -z
    c6 = {minx, maxy, minz} # -x, +y, -z
    c7 = min # -x, -y, -z
    c8 = {maxx, miny, minz} # +x, -y, -z

    # rotate to match world frame
    c1p = Quatern.transform_vector(orientation, c1)
    c2p = Quatern.transform_vector(orientation, c2)
    c3p = Quatern.transform_vector(orientation, c3)
    c4p = Quatern.transform_vector(orientation, c4)
    c5p = Quatern.transform_vector(orientation, c5)
    c6p = Quatern.transform_vector(orientation, c6)
    c7p = Quatern.transform_vector(orientation, c7)
    c8p = Quatern.transform_vector(orientation, c8)

    # calculate new AABB
    {{minxp, minyp, minzp},{maxxp, maxyp, maxzp}} =[c1p,c2p,c3p,c4p,c5p,c6p,c7p,c8p]
    |> Enum.reduce({{@near_infinite, @near_infinite, @near_infinite}, {-@near_infinite,-@near_infinite,-@near_infinite}}, fn({x,y,z},{{minx, miny, minz},{maxx,maxy,maxz}}) ->
      {
        {min(x, minx), min(y, miny), min(z, minz)},
        {max(x, maxx), max(y, maxy), max(z, maxz)}
      }
    end)

    # offset by position
    aabb(
      min: {minxp + px, minyp + py, minzp + pz},
      max: {maxxp + px, maxyp + py, maxzp + pz}
    )
  end

  @doc """
  Creates an AABB from a geometry record, in local space centered on origin.

  Width is the length on the x-axis, height is the length on the y-axis, and depth is the length on the z-axis.

  ## Examples


    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Box, as: Box
    iex> AABB.create_local_from_shape( Box.box(width: 1.0, height: 2.0, depth: 3.0))
    {:aabb, {-0.5, -1.0, -1.5}, {0.5, 1.0, 1.5}}

    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> AABB.create_local_from_shape( Sphere.sphere(radius: 1.0))
    {:aabb, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}}
    iex> AABB.create_local_from_shape( Sphere.sphere(radius: 2.0))
    {:aabb, {-2.0, -2.0, -2.0}, {2.0, 2.0, 2.0}}

    iex> require ElixirRigidPhysics.Collision.AABB, as: AABB
    iex> require ElixirRigidPhysics.Geometry.Capsule, as: Capsule
    iex> AABB.create_local_from_shape( Capsule.capsule(axial_length: 4.0, cap_radius: 1.0))
    {:aabb, {-1.0, -3.0, -1.0}, {1.0, 3.0, 1.0}}

  """
  def create_local_from_shape(Box.box(width: w, height: h, depth: d)) do
    aabb(
      min: {-w / 2, -h / 2, -d / 2},
      max: {w / 2, h / 2, d / 2}
    )
  end

  def create_local_from_shape(Sphere.sphere(radius: r)) do
    aabb(
      min: {-r, -r, -r},
      max: {r, r, r}
    )
  end

  def create_local_from_shape(Capsule.capsule(axial_length: al, cap_radius: cr)) do
    half_height = cr + al / 2.0

    aabb(
      min: {-cr, -half_height, -cr},
      max: {cr, half_height, cr}
    )
  end
end
