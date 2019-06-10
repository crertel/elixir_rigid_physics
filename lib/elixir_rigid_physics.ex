defmodule ElixirRigidPhysics do
  @moduledoc """
  Documentation for ElixirRigidPhysics.
  """

  use GenServer
  alias ElixirRigidPhysics.World

  #######################
  ## Client

  def start_link(%World{} = world) do
    GenServer.start_link(__MODULE__, {world, MapSet.new()})
  end

  def start_link(), do: start_link( %World{})

  def get_world_state(pid) do
    GenServer.call(pid, :get_world_state)
  end

  @doc """
  Steps the world simulation.

  Valid options are:

  * `timestep` is the number of seconds to step the world by.
  """
  @spec step_world(pid(), Keyword.t()) :: :ok
  def step_world(pid, opts \\ []) do
    GenServer.call(pid, {:step_world, opts})
  end

  def add_body_to_world(pid, body) do
    GenServer.call(pid, {:add_body_to_world, body})
  end

  def subscribe_to_world_updates(pid) do
    GenServer.call(pid, :subscribe_to_world_updates)
  end
  def unsubscribe_from_world_updates(pid) do
    GenServer.call(pid, :unsubscribe_from_world_updates)
  end

  #################################
  ## Server

  @impl true
  def init({%World{} = world, subscribers}) do
    {:ok, {world,subscribers}}
  end

  @impl true
  def handle_call(:get_world_state, _from, {%World{} = world, subscribers}) do
    {:reply, world, {world,subscribers}}
  end

  @impl true
  def handle_call({:step_world, opts}, _from, {%World{} = world, subscribers}) do
    dt = Keyword.get(opts, :dt, 1/60)
    new_world = ElixirRigidPhysics.Dynamics.step(world, dt)

    MapSet.to_list(subscribers)
    |> Enum.map( fn(subscriber_pid) ->
      send(subscriber_pid, {:world_update, new_world})
    end)

    {:reply, new_world, {new_world, subscribers}}
  end

  @impl true
  def handle_call({:add_body_to_world, body}, _from, {%World{bodies: bodies} = world, subscribers}) do
    body_ref = make_ref()
    new_world = %World{world | bodies: Map.put(bodies, body_ref, body)}
    {:reply, new_world, {new_world, subscribers}}
  end

  @impl true
  def handle_call(:subscribe_to_world_updates, {from_pid, _}, {world, subscribers}) do
    {:reply, :ok, {world, MapSet.put(subscribers, from_pid)}}
  end

  @impl true
  def handle_call(:unsubscribe_from_world_updates, {from_pid,_}, {world, subscribers}) do
    {:reply, :ok, {world, MapSet.delete(subscribers, from_pid)}}
  end
end
