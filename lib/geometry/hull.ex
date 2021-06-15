defmodule ElixirRigidPhysics.Geometry.Hull do
  @moduledoc """
  Hull geometry module.

  Hulls are lists of coplanar faces wound CCW (to find normal, follow right-hand rule).

  Their center is assumed to be at the origin, but in the absence of a frame that doesn't matter.
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:hull, faces: [], verts: [])
  @type hull_face :: [Vec3.vec3()]
  @type hull :: record(:hull, faces: [hull_face], verts: [Vec3.vec3()])

  @doc """
  Creates a hull geometry in the shape of a box.
  """
  @spec create_box(number, number, number) :: hull
  def create_box(w, h, d) do
    hw = w / 2.0
    hh = h / 2.0
    hd = d / 2.0

    top = [{hw, hh, hd}, {hw, hh, -hd}, {-hw, hh, -hd}, {-hw, hh, hd}]
    bottom = [{hw, -hh, hd}, {-hw, -hh, hd}, {-hw, -hh, -hd}, {hw, -hh, -hd}]
    front = [{hw, hh, hd}, {-hw, hh, hd}, {-hw, -hh, hd}, {hw, -hh, hd}]
    back = [{hw, hh, -hd}, {hw, -hh, -hd}, {-hw, -hh, -hd}, {-hw, hh, -hd}]
    left = [{-hw, hh, hd}, {-hw, hh, -hd}, {-hw, -hh, -hd}, {-hw, -hh, hd}]
    right = [{hw, hh, hd}, {hw, -hh, hd}, {hw, -hh, -hd}, {hw, hh, -hd}]

    hull(
      faces: [
        top,
        bottom,
        front,
        back,
        left,
        right
      ],
      verts: [
        {hw, hh, hd},
        {hw, -hh, hd},
        {-hw, hh, hd},
        {-hw, -hh, hd},
        {hw, hh, -hd},
        {hw, -hh, -hd},
        {-hw, hh, -hd},
        {-hw, -hh, -hd}
      ]
    )
  end

  @near_infinite 1.0e280

  @doc """
  Finds the support point (for GJK usually) of a convex hull.

  Again, we thank Danial Chappius and Reactphysics3d, and also Shiny PIxel (see [point cloud support section](http://vec3.ca/gjk/)).

  ## Examples
    iex> ElixirRigidPhysics.Geometry.Hull, as: Hull
    iex> h = Hull.create_box(1,1,1)
    iex> Hull.support_point(hull, {0.2,0.2,0.2})
    {1.0,1.0,1.0}

    iex> ElixirRigidPhysics.Geometry.Hull, as: Hull
    iex> h = Hull.create_box(1,1,1)
    iex> Hull.support_point(hull, {-0.15,-0.15,-0.25})
    {-1.0,-1.0,-1.0}
  """
  @spec support_point(hull, Vec3.vec3()) :: Vec3.vec3()
  def support_point(hull(verts: verts), direction) do
    {_max_dot_product, best_vert} =
      Enum.reduce(
        verts,
        {-@near_infinite, {-@near_infinite, -@near_infinite, -@near_infinite}},
        fn vert, {best_dot, _best_guess} = acc ->
          dot = Vec3.dot(direction, vert)

          if dot > best_dot do
            {dot, vert}
          else
            acc
          end
        end
      )

    best_vert
  end
end
