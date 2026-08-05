package org.firstinspires.ftc.teamcode.statemachine;
import android.util.Log;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

public class Statemachine <E extends Enum<?>> {

    private E currentState = null;
    private E initialState;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    private final Map<EventType, Map<E, EventHandler<E>>> events = new HashMap<>();

    private final List<StateChangeListener<E>> stateChangeListener = new ArrayList<>();

    public Statemachine(E initialState) {
        this.initialState = initialState;
    }

    public void init() {
        currentState = initialState;
        _update(true);
    }

    public void onStateEnter(E state, EventHandler<E> handler) {
        events.computeIfAbsent(EventType.ENTER, __ -> new HashMap<>()).put(state, handler);
    }
    public void onStateExit(E state, EventHandler<E> handler) {
        events.computeIfAbsent(EventType.EXIT, __ -> new HashMap<>()).put(state, handler);
    }
    public void onStateUpdate(E state, EventHandler<E> handler) {
        events.computeIfAbsent(EventType.UPDATE, __ -> new HashMap<>()).put(state, handler);
    }

    public void onStateEnter(E state, EventHandlerNoParameters<E> handler) {
        events.computeIfAbsent(EventType.ENTER, __ -> new HashMap<>()).put(state, (__, ___) -> handler.handle());
    }
    public void onStateExit(E state, EventHandlerNoParameters<E> handler) {
        events.computeIfAbsent(EventType.EXIT, __ -> new HashMap<>()).put(state, (__, ___) -> handler.handle());
    }
    public void onStateUpdate(E state, EventHandlerNoParameters<E> handler) {
        events.computeIfAbsent(EventType.UPDATE, __ -> new HashMap<>()).put(state, (__, ___) -> handler.handle());
    }

    public void onStateEnter(E state, EventHandlerNoParametersNoTransition<E> handler) {
        events.computeIfAbsent(EventType.ENTER, __ -> new HashMap<>()).put(state, (__, ___) -> {
            return null;
        });
    }

    public void onStateExit(E state, EventHandlerNoParametersNoTransition<E> handler) {
        events.computeIfAbsent(EventType.EXIT, __ -> new HashMap<>()).put(state, (__, ___) -> {
            return null;
        });
    }

    public void onStateUpdate(E state, EventHandlerNoParametersNoTransition<E> handler) {
        events.computeIfAbsent(EventType.UPDATE, __ -> new HashMap<>()).put(state, (__, ___) -> {
            return null;
        });
    }


    public void onStateUpdateDoChange(E state, E to, Supplier<Boolean> condition) {
        events.computeIfAbsent(EventType.UPDATE, __ -> new HashMap<>()).put(state, (__, ___) -> condition.get() ? to : null);
    }
    public void onStateEnterDoChange(E state, E to, Supplier<Boolean> condition) {
        events.computeIfAbsent(EventType.ENTER, __ -> new HashMap<>()).put(state, (__, ___) -> condition.get() ? to : null);
    }

    public void onStateUpdateDoChange(E state, E to, EventHandlerTransition condition) {
        events.computeIfAbsent(EventType.UPDATE, __ -> new HashMap<>()).put(state, (__, time) -> condition.handle(time) ? to : null);
    }
    public void onStateEnterDoChange(E state, E to, EventHandlerTransition condition) {
        events.computeIfAbsent(EventType.ENTER, __ -> new HashMap<>()).put(state, (__, time) -> condition.handle(time) ? to : null);
    }


    public void onAnyStateChange(StateChangeListener<E> listener) {
        stateChangeListener.add(listener);
    }

    public void update() {
        _update(false);
    }

    private void _update(boolean fst) {
        if (currentState == null)
            return;


        E nextState = null;
        while (true) {
            Log.d("org/firstinspires/ftc/teamcode/statemachine", "current state " + currentState);
            if (nextState != null) {
                dispatchEvent(EventType.EXIT);
                E lastState = currentState;
                currentState = nextState;
                timer.reset();
                for (StateChangeListener<E> l : stateChangeListener) l.handle(lastState, nextState, (long) timer.milliseconds());
                fst = true;
            }

            if (fst) {
                fst = false;
                Log.d("org/firstinspires/ftc/teamcode/statemachine", "new state " + nextState);
                nextState = dispatchEvent(EventType.ENTER);
                if (nextState != null)
                    continue;
            }

            nextState = dispatchEvent(EventType.UPDATE);

            if (nextState == null)
                break;
        }
    }

    private E dispatchEvent(EventType type) {
        if (!events.containsKey(type))
            return null;
        if (!Objects.requireNonNull(events.get(type)).containsKey(currentState))
            return null;

        E newState = Objects.requireNonNull(Objects.requireNonNull(events.get(type)).get(currentState)).handle(currentState, (long)timer.milliseconds());
        return type.allowTransitions() ? newState : null;
    }

    public E getCurrentState() {
        return currentState;
    }

    private enum EventType {
        ENTER(true), UPDATE(true), EXIT(false);

        private final boolean allowTransitions;

        public boolean allowTransitions() {
            return allowTransitions;
        }

        EventType(boolean allowTransitions) {
            this.allowTransitions = allowTransitions;
        }

    }

    public interface EventHandler<EE> {
        /**
         *
         * @return the new state the machine should transition, or null if no transition is required.
         * If called for an exit event, return values is ignored.
         */
        EE handle(EE current, long timeSinceTransition);
    }

    public interface EventHandlerNoParameters<EE> {
        /**
         *
         * @return the new state the machine should transition, or null if no transition is required.
         * If called for an exit event, return values is ignored.
         */
        EE handle();
    }

    public interface EventHandlerNoParametersNoTransition<EE> {
        /**
         *
         * @return the new state the machine should transition, or null if no transition is required.
         * If called for an exit event, return values is ignored.
         */
        void handle() throws InterruptedException;
    }

    public interface EventHandlerNoTransition<EE> {
        void handle(EE current, long timeSinceTransition);
    }

    public interface EventHandlerTransition {
        boolean handle(long timeSinceTransition);
    }

    public interface StateChangeListener<EE> {
        void handle(EE last, EE current, long timeSinceTransition);
    }


}