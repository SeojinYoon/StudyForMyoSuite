import jax
import jax.numpy as jnp
import optax

# 1. 1-step 물리 업데이트 함수
def step_physics(state, _):
    x, v = state
    g = -9.81
    dt = 0.1
    
    v_next = v + g * dt
    x_next = x + v_next * dt
    
    return (x_next, v_next), None

# 2. N-step 물리 시뮬레이터 (x0, v0 두 개의 초기값을 받음)
def simulate(params, steps=20):
    x0, v0 = params  # params = [x0, v0]
    initial_state = (x0, v0)
    
    (final_x, final_v), _ = jax.lax.scan(step_physics, initial_state, None, length=steps)
    return final_x

# 3. 손실 함수 (Loss Function - 1초 후 위치와 2초 후 위치를 함께 비교)
def loss_fn(params, target_x_1s, target_x_2s):
    pred_x_1s = simulate(params, steps=10) # 1초 뒤 위치
    pred_x_2s = simulate(params, steps=20) # 2초 뒤 위치
    return (pred_x_1s - target_x_1s)**2 + (pred_x_2s - target_x_2s)**2

# --- 실행 부분 ---

# A. Ground Truth (정답): 높이 10m에서 위로 5m/s 속도로 던진 공
true_x0 = 10.0
true_v0 = 5.0
true_params = jnp.array([true_x0, true_v0])

# 정답 매개변수로 1초 뒤, 2초 뒤 도착 위치 계산 (Target 데이터 준비)
target_x_1s = simulate(true_params, steps=10)
target_x_2s = simulate(true_params, steps=20)

print(f"🎯 Target 위치 (1초 뒤 공의 위치): {target_x_1s:.2f}m")
print(f"🎯 Target 위치 (2초 뒤 공의 위치): {target_x_2s:.2f}m")

# B. 최적화기 및 초기 추정값 설정
# 파라미터 Vector [x0, v0]
estimated_params = jnp.array([0.0, 0.0])  # 초기 추측: 위치 0m, 속도 0m/s

optimizer = optax.adam(learning_rate=0.5)
opt_state = optimizer.init(estimated_params)

loss_and_grad_fn = jax.jit(jax.value_and_grad(loss_fn))

print("\n--- 🔄 JAX Optimization Loop (x0, v0 동시 추정) ---")
for epoch in range(1, 401):
    loss_val, grads = loss_and_grad_fn(estimated_params, target_x_1s, target_x_2s)
    
    updates, opt_state = optimizer.update(grads, opt_state)
    estimated_params = optax.apply_updates(estimated_params, updates)
    
    if epoch % 50 == 0:
        est_x, est_v = estimated_params
        print(f"Epoch {epoch:3d} | Loss: {loss_val:.6f} | 추정 x0: {est_x:.2f}m, v0: {est_v:.2f}m/s")

print(f"\n✅ 최적화 완료!")
print(f"추정 결과 -> 초기 위치 x0: {estimated_params[0]:.2f}m (정답: {true_x0:.2f}m)")
print(f"추정 결과 -> 초기 속도 v0: {estimated_params[1]:.2f}m/s (정답: {true_v0:.2f}m/s)")