defmodule ElixirRigidPhysics.Collision.Intersection.SphereHull do
  @moduledoc """
  Module for sphere-hull intersection tests.
  """

  require ElixirRigidPhysics.Dynamics.Body, as: Body
  require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
  require ElixirRigidPhysics.Geometry.Hull, as: Hull
  require ElixirRigidPhysics.Collision.Contact, as: Contact
  alias Graphmath.Vec3
  alias Graphmath.Quatern

  @verysmol 1.0e-12

  @doc """
  Tests intersections of two bodies.

  ## Examples
    iex> # IO.puts("Test sphere-hull disjoint with no orientation.")
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Geometry.Hull, as: Hull
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> alias ElixirRigidPhysics.Collision.Intersection.SphereHull
    iex> s = Sphere.create(0.5)
    iex> h = Hull.create_box(1,2,3)
    iex> b1 = Body.create(s, position: {0.0,10.0,0.0})
    iex> b2 = Body.create(h, position: {0.0,0.0,0.0})
    iex> SphereHull.check(b1,b2)
    :no_intersection

    iex> # IO.puts("Test sphere-hull in shallow contact with no orientation."
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Geometry.Hull, as: Hull
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> alias ElixirRigidPhysics.Collision.Intersection.SphereHull
    iex> s = Sphere.create(0.5)
    iex> h = Hull.create_box(1,2,3)
    iex> b1 = Body.create(s, position: {0.0,1.25,0.0})
    iex> b2 = Body.create(h, position: {0.0,0.0,0.0})
    iex> SphereHull.check(b1,b2)
    :wat

    iex> # IO.puts("Test sphere-hull in deep contact with no orientation."
    iex> require ElixirRigidPhysics.Geometry.Sphere, as: Sphere
    iex> require ElixirRigidPhysics.Geometry.Hull, as: Hull
    iex> require ElixirRigidPhysics.Dynamics.Body, as: Body
    iex> alias ElixirRigidPhysics.Collision.Intersection.SphereHull
    iex> s = Sphere.create(0.5)
    iex> h = Hull.create_box(1,2,3)
    iex> b1 = Body.create(s, position: {0.0,0.25,0.0})
    iex> b2 = Body.create(h, position: {0.0,0.0,0.0})
    iex> SphereHull.check(b1,b2)
    :wat
  """
  @spec check(Body.body(), Body.body()) :: Contact.contact_result
  def check(
        Body.body(shape: Sphere.sphere(radius: r_a) = sphere, position: p_a),
        Body.body(shape: Hull.hull(faces: faces) = hull, position: p_b, orientation: o_b)
      ) do

    o_a = Quatern.identity()

    o_b_inv = Quatern.conjugate(o_b)

    dir_local_b_to_local_a = Quatern.multiply(o_b_inv, o_a)

    v1 = {0.0, 1.0, 0.0}
    c = support(v1, sphere, p_a, hull, p_b, o_b)

    if Vec3.dot(c, v1) < 0 do
      :no_intersection
    else
      v2 = Vec3.scale(c, -1.0)
      b = support(v2, sphere, p_a, hull, p_b, o_b)
      if Vec3.dot(b, v2) < 0 do
        :no_intersection
      else
        v3 = cross_aba( Vec3.subtract(c, b), Vec3.scale(b, -1.0) )

        do_gjk(v3, {b, c}, sphere, p_a, hull, p_b, o_b, 32)
      end
    end
  end

  def support(dir, sphere, _p_a, hull, _p_b, _o_b) do
    Vec3.subtract( Sphere.support_point(sphere, dir), Hull.support_point(hull, Vec3.scale(dir, -1.0)))
  end

  def do_gjk( dir, simplex, Sphere.sphere(radius: r_a) = sphere, p_a, Hull.hull(faces: faces) = hull, p_b, o_b, 0), do: :intersection
  def do_gjk( dir, simplex, Sphere.sphere(radius: r_a) = sphere, p_a, Hull.hull(faces: faces) = hull, p_b, o_b, iterations_left) do
    a = support(dir, sphere, p_a, hull, p_b, o_b)
    if Vec3.dot(a, dir) < 0 do
      :no_intersection
    else
      case update_simplex(a,simplex) do
        {new_dir, new_simplex} -> do_gjk( new_dir, new_simplex, Sphere.sphere(radius: r_a) = sphere, p_a, Hull.hull(faces: faces) = hull, p_b, o_b, iterations_left - 1)
        :no_intersection -> :no_intersection
      end
    end
  end

  def cross_aba(a, b) do
    Vec3.cross(a, b)
    |> Vec3.cross(a)
  end

  def update_simplex(a, {b,c}) do
    ao = Vec3.negate(a)
    ab = Vec3.subtract(b, a)
    ac = Vec3.subtract(c, a)
    abc = Vec3.cross(ab,ac)
    abp = Vec3.cross( ab, abc)

    if Vec3.dot(abp, ao) > 0 do
      {cross_aba(ab, ao), {a,b}}
    else
      acp = Vec3.cross(abc, ac)
      if Vec3.dot(acp,ao) > 0 do
        { cross_aba(ac,ao), {a,c}}
      else
        if  Vec3.dot(abc,ao) > 0 do
          {abc, a,b,c}
        else
          { Vec3.negate(abc), {a,c,b} }
        end
      end
    end
  end

  def update_simplex(a, {b,c,d}) do
    ao = Vec3.negate(a)
    ab = Vec3.subtract(b, a)
    ac = Vec3.subtract(c,a)
    ad = Vec3.sutbract(d,a)

    abc = Vec3.cross(ab,ac)
    acd = Vec3.cross(ac,ad)
    adb = Vec3.cross(ad,ab)


  end


end
