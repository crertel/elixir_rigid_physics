defmodule ElixirRigidPhysics.Geometry.Hull do
  @moduledoc """
  Hull geometry module.

  Hulls are lists of coplanar faces wound CCW (to find normal, follow right-hand rule).

  Their center is assumed to be at the origin, but in the absence of a frame that doesn't matter.
  """
  alias Graphmath.Vec3

  require Record
  Record.defrecord(:hull, faces: [])
  @type hull_face :: [Vec3.vec3]
  @type hull :: record(:hull, faces: [hull_face])

  @doc """
  Creates a hull geometry.

  Given a list of faces (wound CCW), creates a hull geometry.
  """
  @spec create([hull_face]) :: hull
  def create(faces), do: hull(faces: faces)

  @doc """
  Creates a hull geometry in the shape of a box.
  """
  @spec create_box( number, number, number) :: hull
  def create_box(w, h, d) do

    hw = w/2.0
    hh = h/2.0
    hd = d/2.0

    top = [{hw,hh,hd}, {hw,hh,-hd}, {-hw,hh,-hd}, {-hw,hh,hd}]
    bottom = [{hw,-hh,hd}, {-hw,-hh,hd}, {-hw,-hh,-hd}, {hw,-hh,-hd} ]
    front = [{hw,hh,hd}, {-hw,hh,hd}, {-hw,-hh,hd}, {hw,-hh,hd}]
    back = [{hw,hh,-hd}, {hw,-hh,-hd}, {-hw,-hh,-hd}, {-hw,hh,-hd}]
    left = [{-hw,hh,hd}, {-hw,hh,-hd}, {-hw,-hh,-hd}, {-hw,-hh,hd}]
    right = [{hw,hh,hd}, {hw,-hh,hd}, {hw,-hh,-hd}, {hw,hh,-hd} ]

    hull(faces: [
      top,
      bottom,
      front,
      back,
      left,
      right
    ])
  end
end
