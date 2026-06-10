from datetime import datetime

import altair as alt
import pandas as pd
import streamlit as st

from conector_opcua import OPCConector


st.set_page_config(
    page_title="Controlador federado - OPC UA",
    layout="wide",
)

INTERVALO_REAL_ESPERADO_S = 0.1
MINUTOS_SIM_POR_STEP = 1
MAX_FILAS_HISTORIAL = 36000


def inicializar_estado():
    defaults = {
        "opc": None,
        "conectado": False,
        "historial": [],
        "ultimo_error": "",
        "ip_actual": "",
        "puerto_actual": 0,
        "ultima_actualizacion": None,
        "esperar_step": False,
        "recarga_tiempo_real": False,
        "intervalo_recarga": INTERVALO_REAL_ESPERADO_S,
    }
    for key, value in defaults.items():
        st.session_state.setdefault(key, value)


def desconectar():
    opc = st.session_state.get("opc")
    if opc is not None:
        try:
            opc.desconectar()
        except Exception:
            pass
    st.session_state.opc = None
    st.session_state.conectado = False
    st.session_state.recarga_tiempo_real = False


def conectar(ip, puerto):
    desconectar()
    opc = OPCConector(ip, puerto)
    opc.conectar()
    st.session_state.opc = opc
    st.session_state.conectado = True
    st.session_state.ip_actual = ip
    st.session_state.puerto_actual = puerto
    st.session_state.ultimo_error = ""


def registrar_muestra(estado):
    timestamp = datetime.now()
    step = estado["step"]
    minuto_simulado = step * MINUTOS_SIM_POR_STEP
    exterior = estado["temperatura_exterior"]

    for i, temperatura in enumerate(estado["temperaturas"], start=1):
        voltaje = estado["voltajes"][i - 1] if i - 1 < len(estado["voltajes"]) else None
        st.session_state.historial.append(
            {
                "timestamp": timestamp,
                "step": step,
                "minuto_simulado": minuto_simulado,
                "habitacion": f"Habitacion {i}",
                "temperatura": float(temperatura),
                "voltaje": float(voltaje) if voltaje is not None else None,
                "temperatura_exterior": float(exterior),
            }
        )

    st.session_state.historial = st.session_state.historial[-MAX_FILAS_HISTORIAL:]


def leer_opc(esperar_siguiente_step=False):
    opc = st.session_state.opc
    if opc is None:
        return None

    if esperar_siguiente_step:
        opc.wait_for_next_step(timeout_s=2)

    estado = opc.obtener_estado()
    registrar_muestra(estado)
    st.session_state.ultimo_error = ""
    st.session_state.ultima_actualizacion = datetime.now()
    return estado


def dataframe_historial():
    if not st.session_state.historial:
        return pd.DataFrame(
            columns=[
                "timestamp",
                "step",
                "minuto_simulado",
                "habitacion",
                "temperatura",
                "voltaje",
                "temperatura_exterior",
            ]
        )
    df = pd.DataFrame(st.session_state.historial)
    if "minuto_simulado" not in df.columns or df["minuto_simulado"].isna().any():
        df["minuto_simulado"] = df["step"] * MINUTOS_SIM_POR_STEP
    return df


def muestras_unicas(df):
    if df.empty:
        return pd.DataFrame(columns=["timestamp", "step", "minuto_simulado", "temperatura_exterior"])

    return (
        df.drop_duplicates(subset=["timestamp", "step"])
        .sort_values("timestamp")
        .reset_index(drop=True)
    )


def calcular_monitoreo(df):
    muestras = muestras_unicas(df)
    if muestras.empty:
        return {
            "muestras": muestras,
            "n_muestras": 0,
            "step_delta": 0,
            "sim_minutos": 0,
            "real_s": 0.0,
            "intervalo_promedio_s": None,
            "intervalo_ultimo_s": None,
            "frecuencia_hz": None,
            "factor_velocidad": None,
            "atraso_s": None,
            "edad_ultima_s": None,
        }

    step_ini = int(muestras["step"].iloc[0])
    step_fin = int(muestras["step"].iloc[-1])
    step_delta = max(0, step_fin - step_ini)
    sim_minutos = step_delta * MINUTOS_SIM_POR_STEP
    real_s = (muestras["timestamp"].iloc[-1] - muestras["timestamp"].iloc[0]).total_seconds()

    intervalos = muestras["timestamp"].diff().dt.total_seconds().dropna()
    intervalo_promedio_s = float(intervalos.mean()) if not intervalos.empty else None
    intervalo_ultimo_s = float(intervalos.iloc[-1]) if not intervalos.empty else None
    frecuencia_hz = 1 / intervalo_promedio_s if intervalo_promedio_s and intervalo_promedio_s > 0 else None
    factor_velocidad = (sim_minutos * 60) / real_s if real_s > 0 else None
    atraso_s = real_s - (step_delta * INTERVALO_REAL_ESPERADO_S) if step_delta > 0 else None
    edad_ultima_s = (datetime.now() - muestras["timestamp"].iloc[-1]).total_seconds()

    muestras = muestras.copy()
    muestras["intervalo_real_s"] = muestras["timestamp"].diff().dt.total_seconds()
    muestras["minuto_simulado"] = muestras["step"] * MINUTOS_SIM_POR_STEP
    muestras["tiempo_real_s"] = (muestras["timestamp"] - muestras["timestamp"].iloc[0]).dt.total_seconds()

    return {
        "muestras": muestras,
        "n_muestras": len(muestras),
        "step_delta": step_delta,
        "sim_minutos": sim_minutos,
        "real_s": real_s,
        "intervalo_promedio_s": intervalo_promedio_s,
        "intervalo_ultimo_s": intervalo_ultimo_s,
        "frecuencia_hz": frecuencia_hz,
        "factor_velocidad": factor_velocidad,
        "atraso_s": atraso_s,
        "edad_ultima_s": edad_ultima_s,
    }


def graficar_lineas(df, y, titulo, unidad):
    return (
        alt.Chart(df)
        .mark_line(point=True)
        .encode(
            x=alt.X("minuto_simulado:Q", title="Tiempo simulado (min)"),
            y=alt.Y(f"{y}:Q", title=unidad),
            color=alt.Color("habitacion:N", title="Habitacion"),
            tooltip=[
                alt.Tooltip("minuto_simulado:Q", title="Tiempo simulado (min)"),
                alt.Tooltip("timestamp:T", title="Hora lectura"),
                alt.Tooltip("habitacion:N", title="Habitacion"),
                alt.Tooltip(f"{y}:Q", title=titulo, format=".2f"),
            ],
        )
        .properties(height=330)
        .configure_legend(orient="bottom")
    )


def construir_evolucion(df, variables):
    series = []

    if "Temperatura habitaciones" in variables:
        temp_df = df[["timestamp", "step", "minuto_simulado", "habitacion", "temperatura"]].rename(
            columns={"temperatura": "valor", "habitacion": "serie"}
        )
        temp_df["variable"] = "Temperatura habitaciones"
        series.append(temp_df)

    if "Voltajes" in variables:
        volt_df = df[["timestamp", "step", "minuto_simulado", "habitacion", "voltaje"]].rename(
            columns={"voltaje": "valor", "habitacion": "serie"}
        )
        volt_df["variable"] = "Voltajes"
        series.append(volt_df)

    if "Temperatura exterior" in variables:
        exterior_df = df.drop_duplicates(subset=["timestamp", "step"])[
            ["timestamp", "step", "minuto_simulado", "temperatura_exterior"]
        ].rename(columns={"temperatura_exterior": "valor"})
        exterior_df["serie"] = "Exterior"
        exterior_df["variable"] = "Temperatura exterior"
        series.append(exterior_df)

    if not series:
        return pd.DataFrame(columns=["timestamp", "step", "minuto_simulado", "serie", "valor", "variable"])

    return pd.concat(series, ignore_index=True)


inicializar_estado()

st.title("Controlador federado de regulacion de temperatura")
st.caption("Visualizador OPC UA (solo monitoreo): temperaturas de habitaciones, temperatura exterior y voltajes de aire acondicionado.")

with st.sidebar:
    st.header("Conexion OPC UA")
    ip = st.text_input("IP servidor", value=st.session_state.ip_actual or "192.168.1.142")
    puerto = st.number_input(
        "Puerto",
        min_value=1,
        max_value=65535,
        value=st.session_state.puerto_actual or 4840,
        step=1,
    )

    col_a, col_b = st.columns(2)
    with col_a:
        if st.button("Conectar", use_container_width=True):
            try:
                conectar(ip, int(puerto))
                st.toast("Conexion OPC UA establecida.")
            except Exception as exc:
                st.session_state.ultimo_error = f"No se pudo conectar: {exc}"
                st.session_state.conectado = False

    with col_b:
        if st.button("Desconectar", use_container_width=True):
            desconectar()
            st.toast("Cliente desconectado.")

    st.divider()
    st.write("Estado")
    if st.session_state.conectado:
        st.success(f"Conectado a {st.session_state.ip_actual}:{st.session_state.puerto_actual}")
    else:
        st.info("Sin conexion activa.")

    esperar_step = st.toggle("Esperar siguiente minuto simulado", key="esperar_step")

    st.divider()
    st.write("Tiempo real")
    recarga_tiempo_real = st.toggle(
        "Recarga en tiempo real",
        key="recarga_tiempo_real",
        disabled=not st.session_state.conectado,
    )
    intervalo_recarga = st.slider(
        "Intervalo de lectura (s)",
        min_value=0.1,
        max_value=5.0,
        step=0.1,
        format="%.1f",
        key="intervalo_recarga",
        disabled=not recarga_tiempo_real,
    )
    st.caption("Esperado: 0.1 s reales = 1 minuto simulado.")

    if st.button(
        "Actualizar lectura",
        type="primary",
        use_container_width=True,
        disabled=not st.session_state.conectado,
    ):
        try:
            leer_opc(esperar_siguiente_step=esperar_step)
        except Exception as exc:
            st.session_state.ultimo_error = f"No se pudo leer OPC UA: {exc}"

    if st.button("Limpiar historial", use_container_width=True):
        st.session_state.historial = []

run_every = (
    st.session_state.intervalo_recarga
    if st.session_state.conectado and st.session_state.recarga_tiempo_real
    else None
)


@st.fragment(run_every=run_every)
def panel_datos():
    if st.session_state.conectado and st.session_state.recarga_tiempo_real:
        try:
            leer_opc(esperar_siguiente_step=st.session_state.esperar_step)
        except Exception as exc:
            st.session_state.ultimo_error = f"No se pudo leer OPC UA: {exc}"

    if st.session_state.ultimo_error:
        st.error(st.session_state.ultimo_error)

    df = dataframe_historial()
    ultima_lectura = df.sort_values("timestamp").groupby("habitacion").tail(1) if not df.empty else df
    monitoreo = calcular_monitoreo(df)

    metric_cols = st.columns(4)
    if st.session_state.conectado and not ultima_lectura.empty:
        temp_promedio = ultima_lectura["temperatura"].mean()
        temp_min = ultima_lectura["temperatura"].min()
        temp_max = ultima_lectura["temperatura"].max()
        exterior = ultima_lectura["temperatura_exterior"].iloc[-1]
        step = int(ultima_lectura["step"].iloc[-1])

        minuto_simulado = step * MINUTOS_SIM_POR_STEP

        metric_cols[0].metric("Tiempo simulado", f"{minuto_simulado:.0f} min")
        metric_cols[1].metric("Temp. promedio", f"{temp_promedio:.2f} C")
        metric_cols[2].metric("Rango habitaciones", f"{temp_min:.2f} - {temp_max:.2f} C")
        metric_cols[3].metric("Temp. exterior", f"{exterior:.2f} C")
        if st.session_state.ultima_actualizacion is not None:
            st.caption(
                "Ultima actualizacion: "
                f"{st.session_state.ultima_actualizacion.strftime('%H:%M:%S')}"
            )
    else:
        metric_cols[0].metric("Tiempo simulado", "-")
        metric_cols[1].metric("Temp. promedio", "-")
        metric_cols[2].metric("Rango habitaciones", "-")
        metric_cols[3].metric("Temp. exterior", "-")

    tab_resumen, tab_graficos, tab_monitor, tab_datos = st.tabs(
        ["Resumen", "Graficos", "Monitor", "Datos"]
    )

    with tab_resumen:
        cols = st.columns([2, 1])
        with cols[0]:
            st.subheader("Lectura actual por habitacion")
            if ultima_lectura.empty:
                st.info("Conecta el cliente OPC UA y presiona Actualizar lectura para visualizar datos.")
            else:
                st.dataframe(
                    ultima_lectura[
                        [
                            "minuto_simulado",
                            "habitacion",
                            "temperatura",
                            "voltaje",
                            "temperatura_exterior",
                            "timestamp",
                        ]
                    ],
                    use_container_width=True,
                    hide_index=True,
                )

        with cols[1]:
            st.subheader("Voltajes actuales")
            if ultima_lectura.empty:
                st.info("Sin datos.")
            else:
                st.bar_chart(ultima_lectura.set_index("habitacion")["voltaje"])

    with tab_graficos:
        st.subheader("Evolucion de variables")
        cols_config = st.columns([2, 1])
        with cols_config[0]:
            variables = st.multiselect(
                "Variables",
                ["Temperatura habitaciones", "Temperatura exterior", "Voltajes"],
                default=["Temperatura habitaciones", "Temperatura exterior", "Voltajes"],
            )
        with cols_config[1]:
            st.metric("Equivalencia", f"{MINUTOS_SIM_POR_STEP} min / paso")

        if df.empty:
            st.info("Aun no hay historial para graficar.")
        else:
            evolucion_df = construir_evolucion(df, variables)
            if evolucion_df.empty:
                st.info("Selecciona al menos una variable para graficar.")
            else:
                evolucion_chart = (
                    alt.Chart(evolucion_df)
                    .mark_line(point=True)
                    .encode(
                        x=alt.X("minuto_simulado:Q", title="Tiempo simulado (min)"),
                        y=alt.Y("valor:Q", title="Valor"),
                        color=alt.Color("serie:N", title="Serie"),
                        strokeDash=alt.StrokeDash("variable:N", title="Variable"),
                        tooltip=[
                            alt.Tooltip("minuto_simulado:Q", title="Tiempo simulado (min)"),
                            alt.Tooltip("timestamp:T", title="Hora lectura"),
                            alt.Tooltip("variable:N", title="Variable"),
                            alt.Tooltip("serie:N", title="Serie"),
                            alt.Tooltip("valor:Q", title="Valor", format=".2f"),
                        ],
                    )
                    .properties(height=360)
                    .configure_legend(orient="bottom")
                )
                st.altair_chart(evolucion_chart, use_container_width=True)

            cols = st.columns(2)
            with cols[0].container(border=True):
                st.subheader("Temperatura habitaciones")
                st.altair_chart(
                    graficar_lineas(df, "temperatura", "Temperatura", "Temperatura (C)"),
                    use_container_width=True,
                )

            with cols[1].container(border=True):
                st.subheader("Voltajes aire acondicionado")
                st.altair_chart(graficar_lineas(df, "voltaje", "Voltaje", "Voltaje"), use_container_width=True)

            st.subheader("Temperatura exterior")
            exterior_df = df.drop_duplicates(subset=["timestamp", "step"])[
                ["timestamp", "step", "minuto_simulado", "temperatura_exterior"]
            ]
            exterior_chart = (
                alt.Chart(exterior_df)
                .mark_line(point=True, color="#2a9d8f")
                .encode(
                    x=alt.X("minuto_simulado:Q", title="Tiempo simulado (min)"),
                    y=alt.Y("temperatura_exterior:Q", title="Temperatura exterior (C)"),
                    tooltip=[
                        alt.Tooltip("minuto_simulado:Q", title="Tiempo simulado (min)"),
                        alt.Tooltip("timestamp:T", title="Hora lectura"),
                        alt.Tooltip("temperatura_exterior:Q", title="Exterior", format=".2f"),
                    ],
                )
                .properties(height=280)
            )
            st.altair_chart(exterior_chart, use_container_width=True)

    with tab_monitor:
        st.subheader("Monitor de ritmo de simulacion")
        st.caption("Referencia del servidor: 0.1 s reales por muestra y 1 minuto simulado por cada paso.")

        monitor_cols = st.columns(4)
        intervalo_promedio = monitoreo["intervalo_promedio_s"]
        intervalo_ultimo = monitoreo["intervalo_ultimo_s"]
        factor_velocidad = monitoreo["factor_velocidad"]
        atraso_s = monitoreo["atraso_s"]

        monitor_cols[0].metric(
            "Intervalo promedio",
            f"{intervalo_promedio:.3f} s" if intervalo_promedio is not None else "-",
            delta=(
                f"{intervalo_promedio - INTERVALO_REAL_ESPERADO_S:+.3f} s"
                if intervalo_promedio is not None
                else None
            ),
        )
        monitor_cols[1].metric(
            "Ultimo intervalo",
            f"{intervalo_ultimo:.3f} s" if intervalo_ultimo is not None else "-",
        )
        monitor_cols[2].metric(
            "Avance simulado",
            f"{monitoreo['sim_minutos']:.0f} min",
            delta=f"{monitoreo['step_delta']} min desde la primera muestra",
        )
        monitor_cols[3].metric(
            "Factor velocidad",
            f"{factor_velocidad:.0f}x" if factor_velocidad is not None else "-",
            delta=(
                f"{factor_velocidad - 600:+.0f}x vs 600x"
                if factor_velocidad is not None
                else None
            ),
        )

        status_cols = st.columns(4)
        status_cols[0].metric("Muestras capturadas", f"{monitoreo['n_muestras']}")
        status_cols[1].metric(
            "Frecuencia lectura",
            f"{monitoreo['frecuencia_hz']:.2f} Hz" if monitoreo["frecuencia_hz"] is not None else "-",
        )
        status_cols[2].metric(
            "Tiempo real observado",
            f"{monitoreo['real_s']:.2f} s",
        )
        status_cols[3].metric(
            "Desfase acumulado",
            f"{atraso_s:+.3f} s" if atraso_s is not None else "-",
        )

        if monitoreo["edad_ultima_s"] is not None:
            if monitoreo["edad_ultima_s"] <= 2 * INTERVALO_REAL_ESPERADO_S:
                st.success(f"Ultima muestra recibida hace {monitoreo['edad_ultima_s']:.3f} s.")
            else:
                st.warning(f"Ultima muestra recibida hace {monitoreo['edad_ultima_s']:.3f} s.")

        muestras = monitoreo["muestras"]
        if len(muestras) < 2:
            st.info("Se necesitan al menos dos lecturas para calcular intervalos.")
        else:
            intervalos_df = muestras.dropna(subset=["intervalo_real_s"])
            regla_df = pd.DataFrame({"esperado": [INTERVALO_REAL_ESPERADO_S]})
            intervalo_chart = (
                alt.Chart(intervalos_df)
                .mark_line(point=True)
                .encode(
                    x=alt.X("timestamp:T", title="Hora de lectura"),
                    y=alt.Y("intervalo_real_s:Q", title="Intervalo real (s)"),
                    tooltip=[
                        alt.Tooltip("timestamp:T", title="Hora lectura"),
                        alt.Tooltip("minuto_simulado:Q", title="Tiempo simulado (min)"),
                        alt.Tooltip("intervalo_real_s:Q", title="Intervalo", format=".3f"),
                    ],
                )
                .properties(height=260)
            )
            regla_chart = (
                alt.Chart(regla_df)
                .mark_rule(color="#d62728", strokeDash=[6, 4])
                .encode(y=alt.Y("esperado:Q"))
            )
            st.altair_chart((intervalo_chart + regla_chart), use_container_width=True)

            avance_chart = (
                alt.Chart(muestras)
                .mark_line(point=True, color="#2a9d8f")
                .encode(
                    x=alt.X("tiempo_real_s:Q", title="Tiempo real observado (s)"),
                    y=alt.Y("minuto_simulado:Q", title="Minuto simulado"),
                    tooltip=[
                        alt.Tooltip("tiempo_real_s:Q", title="Tiempo real", format=".2f"),
                        alt.Tooltip("minuto_simulado:Q", title="Minuto simulado"),
                    ],
                )
                .properties(height=260)
            )
            st.altair_chart(avance_chart, use_container_width=True)

    with tab_datos:
        st.subheader("Historial OPC UA")
        columnas_visibles = [
            "minuto_simulado",
            "habitacion",
            "temperatura",
            "voltaje",
            "temperatura_exterior",
            "timestamp",
        ]
        st.dataframe(df[columnas_visibles] if not df.empty else df, use_container_width=True, hide_index=True)


panel_datos()