defmodule ElixirRigidPhysics.Dynamics.Body do
  @moduledoc """
  Module to handle bodies, the fundamental unit of physics in ERP.
  """
  alias Graphmath.Quatern
  alias Graphmath.Vec3
  alias ElixirRigidPhysics.Geometry

  require Record

  Record.defrecord(:body,
    shape: nil,
    mass: 0.0,
    position: {0.0, 0.0, 0.0},
    orientation: Quatern.identity(),
    linear_dampening: 0.0,
    angular_dampening: 0.0,
    linear_velocity: {0.0, 0.0, 0.0},
    angular_velocity: {0.0, 0.0, 0.0},
    accumulated_force: {0.0, 0.0, 0.0},
    accumulated_torque: {0.0, 0.0, 0.0}
  )

  @type body ::
          record(:body,
            shape: Geometry.geometry(),
            mass: number,
            position: Vec3.vec3(),
            orientation: Quatern.quatern(),
            linear_dampening: number,
            angular_dampening: number,
            linear_velocity: Vec3.vec3(),
            angular_velocity: Vec3.vec3(),
            accumulated_force: Vec3.vec3(),
            accumulated_torque: Vec3.vec3()
          )

  @spec create(Geometry.geometry(), [{atom, any}]) :: body
  def create(shape, opts \\ []) do
    mass = Keyword.get(opts, :mass, 0)
    position = Keyword.get(opts, :position, {0, 0, 0})
    orientation = Keyword.get(opts, :orientation, Quatern.identity())
    linear_dampening = Keyword.get(opts, :linear_dampening, 0)
    angular_dampening = Keyword.get(opts, :angular_dampening, 0)

    linear_velocity = Keyword.get(opts, :linear_velocity, {0, 0, 0})
    angular_velocity = Keyword.get(opts, :angular_velocity, {0, 0, 0})

    body(
      shape: shape,
      linear_dampening: linear_dampening,
      angular_dampening: angular_dampening,
      position: position,
      orientation: orientation,
      linear_velocity: linear_velocity,
      angular_velocity: angular_velocity,
      mass: mass
    )
  end

  @spec to_map(body) :: map()
  def to_map(
        body(
          shape: shape,
          linear_dampening: linear_dampening,
          angular_dampening: angular_dampening,
          position: position,
          orientation: orientation,
          linear_velocity: linear_velocity,
          angular_velocity: angular_velocity,
          mass: mass
        )
      ) do
    %{
      shape: shape,
      linear_dampening: linear_dampening,
      angular_dampening: angular_dampening,
      position: position,
      orientation: orientation,
      linear_velocity: linear_velocity,
      angular_velocity: angular_velocity,
      mass: mass
    }
  end
end
