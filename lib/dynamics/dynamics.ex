defmodule ElixirRigidPhysics.Dynamics do
  alias ElixirRigidPhysics.World
  alias Graphmath.Mat44
  alias Graphmath.Vec3
  alias Graphmath.Quatern

  def step(%World{timestep: timestep, current_time: current_time, bodies: bodies} = world, dt) do

    new_bodies = for {r, b} <- bodies, into: %{} do

      %{
        position: position,
        orientation: {ow,ox,oy,oz} = orientation,

        linear_velocity: linear_velocity,
        angular_velocity: {avx,avy,avz} = angular_velocity,
      } = b

      # integrate linear velocity
      new_position = Vec3.scale(linear_velocity, dt)|> Vec3.add(position)

      # integrate angular velocity

      half_dt = dt / 2.0
      ang_vel_quat = {0, avx * half_dt, avy * half_dt, avz * half_dt}
      new_orientation_temp = Quatern.multiply(ang_vel_quat, orientation)
      IO.inspect({dt, new_orientation_temp, ang_vel_quat}, label: "ROTROTROT")
      new_orientation = if Quatern.norm(new_orientation_temp) == 0 do
        new_orientation_temp
      else
        new_orientation_temp |> Quatern.normalize()
      end

      #half_dt = dt / 2.0
      #new_orientation = {
      #  -half_dt *((avx * ox) + (avy * oy) + (avz * oz)),
      #  half_dt * ((avx * ow) + (avy * oz) - (avz * oy)),
      #  half_dt * ((avy * ow) + (avz * ox) - (avx * oz)),
      #  half_dt * ((avz * ow) + (avx * oy) - (avy * ox))
      #}

      IO.inspect(new_orientation, label: "ROTROTROT")

      new_body = Map.merge(b, %{
        position: new_position,
        orientation: new_orientation #|> Quatern.normalize()
      })

      {r, new_body}
    end
    %World{world | timestep: timestep + 1, current_time: current_time + dt, bodies: new_bodies}
  end
end
