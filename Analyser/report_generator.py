"""
Report Generator for Physics Engine Analyser
Compiles comprehensive HTML and Markdown diagnostics reports with embedded
screenshot evidence, animated GIF replays, health scores, and actionable engineering fixes.
"""

import os
import json
import base64

class ReportGenerator:
    def __init__(self, output_dir):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    def generate(self, test_name, backend_name, diagnostic_engine, visual_recorder, extra_meta=None):
        """
        Builds both report.html and report.md in output_dir.
        Returns paths: (html_path, md_path)
        """
        extra_meta = extra_meta or {}
        score = diagnostic_engine.health_score
        anomalies = diagnostic_engine.anomalies
        recommendations = diagnostic_engine.recommendations
        screenshots = visual_recorder.saved_screenshots
        gif_path = visual_recorder.get_gif_path()

        # Grade assignment
        if score >= 90:
            grade, grade_col = "A (EXCELLENT)", "#10b981"
        elif score >= 75:
            grade, grade_col = "B (STABLE)", "#3b82f6"
        elif score >= 50:
            grade, grade_col = "C (MARGINAL)", "#f59e0b"
        else:
            grade, grade_col = "F (DEFECTIVE)", "#ef4444"

        # 1. Generate Markdown Report
        md_content = self._build_markdown(
            test_name=test_name,
            backend_name=backend_name,
            score=score,
            grade=grade,
            anomalies=anomalies,
            recommendations=recommendations,
            screenshots=screenshots,
            gif_path=gif_path,
            extra_meta=extra_meta
        )
        md_file = os.path.join(self.output_dir, "report.md")
        with open(md_file, "w", encoding="utf-8") as f:
            f.write(md_content)

        # 2. Generate Interactive HTML Report
        html_content = self._build_html(
            test_name=test_name,
            backend_name=backend_name,
            score=score,
            grade=grade,
            grade_col=grade_col,
            anomalies=anomalies,
            recommendations=recommendations,
            screenshots=screenshots,
            gif_path=gif_path,
            extra_meta=extra_meta
        )
        html_file = os.path.join(self.output_dir, "report.html")
        with open(html_file, "w", encoding="utf-8") as f:
            f.write(html_content)

        return html_file, md_file

    def _build_markdown(self, test_name, backend_name, score, grade, anomalies, recommendations, screenshots, gif_path, extra_meta):
        lines = []
        lines.append(f"# 🛡️ Physics Engine Diagnostic Report")
        lines.append(f"**Test Scenario:** `{test_name}`  ")
        lines.append(f"**Backend:** `{backend_name}`  ")
        lines.append(f"**Physics Health Score:** **{score}/100** (`{grade}`)  ")
        lines.append("")

        # Metadata
        lines.append("## 📊 Run Metrics")
        lines.append(f"| Metric | Value |")
        lines.append(f"|---|---|")
        for k, v in extra_meta.items():
            lines.append(f"| **{k}** | {v} |")
        lines.append(f"| **Anomalies Detected** | {len(anomalies)} |")
        lines.append("")

        # Visual Evidence
        lines.append("## 📸 Visual Evidence")
        if gif_path and os.path.exists(gif_path):
            rel_gif = os.path.basename(gif_path)
            lines.append(f"### 🎞️ Simulation Replay (GIF)\n![Replay]({rel_gif})\n")

        if screenshots:
            lines.append("### 🖼️ Keyframe Captures")
            for label, path in sorted(screenshots.items()):
                rel_path = os.path.basename(path)
                lines.append(f"- **{label.upper()}**: ![{label}]({rel_path})")
            lines.append("")

        # Anomalies
        lines.append("## ⚠️ Detected Physical Anomalies")
        if not anomalies:
            lines.append("> ✅ **No physical anomalies detected.** Simulation exhibited realistic stability, momentum conservation, and contact resolution.")
        else:
            lines.append("| Severity | Code | Time | Observed | Threshold | Description |")
            lines.append("|---|---|---|---|---|---|")
            for a in anomalies:
                sev_icon = "🔴" if a.severity == "CRITICAL" else ("🟡" if a.severity == "WARNING" else "ℹ️")
                lines.append(f"| {sev_icon} **{a.severity}** | `{a.code}` | {a.timestamp:.2f}s | {a.metric_value:.1f} | {a.threshold:.1f} | {a.description} |")
        lines.append("")

        # Recommendations
        lines.append("## 🔧 Prescribed Fixes & Engineering Actions")
        if not recommendations:
            lines.append("> All physical subsystems verified within realistic bounds.")
        else:
            for idx, r in enumerate(recommendations, 1):
                p_icon = "🚨" if r.priority == "CRITICAL" else "⚠️"
                lines.append(f"### {idx}. {p_icon} [{r.priority}] {r.title}")
                lines.append(f"- **Subsystem:** `{r.subsystem}`")
                lines.append(f"- **Root Cause:** {r.root_cause}")
                lines.append("- **Step-by-Step Fix Instructions:**")
                for inst in r.fix_instructions:
                    lines.append(f"  - {inst}")
                lines.append("")

        return "\n".join(lines)

    def _build_html(self, test_name, backend_name, score, grade, grade_col, anomalies, recommendations, screenshots, gif_path, extra_meta):
        # Screenshots HTML cards
        screenshots_html = ""
        for label, path in sorted(screenshots.items()):
            bname = os.path.basename(path)
            screenshots_html += f"""
            <div class="card screenshot-card">
                <div class="card-header">{label.replace('_', ' ').title()}</div>
                <img src="{bname}" alt="{label}" onclick="window.open(this.src)" />
            </div>
            """

        # Replay GIF HTML
        gif_html = ""
        if gif_path and os.path.exists(gif_path):
            rel_gif = os.path.basename(gif_path)
            gif_html = f"""
            <div class="card replay-card">
                <div class="card-header">🎞️ Dynamic Simulation Replay</div>
                <div class="gif-container">
                    <img src="{rel_gif}" alt="Simulation Replay" />
                </div>
            </div>
            """

        # Metric cards
        meta_html = ""
        for k, v in extra_meta.items():
            meta_html += f"""
            <div class="metric-card">
                <div class="metric-label">{k}</div>
                <div class="metric-val">{v}</div>
            </div>
            """

        # Anomalies Rows
        anom_rows = ""
        if not anomalies:
            anom_rows = """<tr><td colspan="5" style="text-align:center; color:#10b981; padding:20px;">
                ✅ Perfect score! No physical defects or instabilities detected.
            </td></tr>"""
        else:
            for a in anomalies:
                badge_class = "badge-crit" if a.severity == "CRITICAL" else ("badge-warn" if a.severity == "WARNING" else "badge-info")
                anom_rows += f"""
                <tr>
                    <td><span class="badge {badge_class}">{a.severity}</span></td>
                    <td><code>{a.code}</code></td>
                    <td>{a.timestamp:.2f}s</td>
                    <td><b>{a.metric_value:.1f}</b> <span class="dim">/ {a.threshold:.1f}</span></td>
                    <td>{a.description}</td>
                </tr>
                """

        # Recommendations HTML
        recs_html = ""
        if not recommendations:
            recs_html = """<div class="clean-box">No action required. Physics parameters meet realism guidelines.</div>"""
        else:
            for idx, r in enumerate(recommendations, 1):
                p_class = "badge-crit" if r.priority == "CRITICAL" else "badge-warn"
                inst_items = "".join(f"<li>{i}</li>" for i in r.fix_instructions)
                recs_html += f"""
                <div class="recommendation-item">
                    <div class="rec-header">
                        <span class="badge {p_class}">{r.priority}</span>
                        <h3>#{idx} {r.title}</h3>
                        <span class="subsystem-pill">{r.subsystem}</span>
                    </div>
                    <p class="root-cause"><b>Root Cause:</b> {r.root_cause}</p>
                    <ul class="fix-list">
                        {inst_items}
                    </ul>
                </div>
                """

        html = f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Physics Engine Report - {test_name}</title>
<style>
    :root {{
        --bg-main: #0b0f19;
        --bg-surface: #111827;
        --bg-card: #1f2937;
        --border-color: #374151;
        --text-main: #f3f4f6;
        --text-dim: #9ca3af;
        --accent-blue: #3b82f6;
        --accent-emerald: #10b981;
        --accent-amber: #f59e0b;
        --accent-rose: #ef4444;
    }}
    * {{ box-sizing: border-box; margin: 0; padding: 0; }}
    body {{
        font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
        background-color: var(--bg-main);
        color: var(--text-main);
        line-height: 1.5;
        padding: 30px;
    }}
    .container {{ max-width: 1300px; margin: 0 auto; }}
    header {{
        display: flex;
        justify-content: space-between;
        align-items: center;
        padding-bottom: 24px;
        border-bottom: 1px solid var(--border-color);
        margin-bottom: 30px;
    }}
    .brand h1 {{ font-size: 26px; font-weight: 700; color: #fff; }}
    .brand p {{ color: var(--text-dim); font-size: 14px; margin-top: 4px; }}
    .score-badge {{
        text-align: right;
        background: var(--bg-surface);
        padding: 14px 24px;
        border-radius: 12px;
        border: 2px solid {grade_col};
        box-shadow: 0 4px 16px rgba(0,0,0,0.4);
    }}
    .score-val {{ font-size: 32px; font-weight: 800; color: {grade_col}; }}
    .score-grade {{ font-size: 13px; font-weight: 600; text-transform: uppercase; color: var(--text-dim); }}
    
    .metrics-grid {{
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
        gap: 16px;
        margin-bottom: 30px;
    }}
    .metric-card {{
        background: var(--bg-surface);
        border: 1px solid var(--border-color);
        border-radius: 10px;
        padding: 16px 20px;
    }}
    .metric-label {{ font-size: 12px; text-transform: uppercase; color: var(--text-dim); letter-spacing: 0.5px; }}
    .metric-val {{ font-size: 22px; font-weight: 700; color: #fff; margin-top: 6px; }}

    h2 {{ font-size: 20px; font-weight: 600; margin: 30px 0 16px; display: flex; align-items: center; gap: 8px; }}
    
    .media-section {{
        display: grid;
        grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
        gap: 20px;
        margin-bottom: 30px;
    }}
    .card {{
        background: var(--bg-surface);
        border: 1px solid var(--border-color);
        border-radius: 10px;
        overflow: hidden;
    }}
    .card-header {{
        padding: 10px 16px;
        background: var(--bg-card);
        font-size: 13px;
        font-weight: 600;
        border-bottom: 1px solid var(--border-color);
    }}
    .card img {{
        width: 100%;
        height: auto;
        display: block;
        cursor: pointer;
        transition: transform 0.2s ease;
    }}
    .card img:hover {{ transform: scale(1.02); }}

    .table-container {{
        background: var(--bg-surface);
        border: 1px solid var(--border-color);
        border-radius: 10px;
        overflow-x: auto;
        margin-bottom: 30px;
    }}
    table {{ width: 100%; border-collapse: collapse; text-align: left; font-size: 14px; }}
    th, td {{ padding: 12px 16px; border-bottom: 1px solid var(--border-color); }}
    th {{ background: var(--bg-card); font-weight: 600; color: var(--text-dim); }}
    code {{ background: #26334a; padding: 2px 6px; border-radius: 4px; font-family: Consolas, monospace; font-size: 13px; color: #60a5fa; }}
    .dim {{ color: var(--text-dim); }}

    .badge {{
        display: inline-block;
        padding: 4px 10px;
        border-radius: 9999px;
        font-size: 11px;
        font-weight: 700;
        letter-spacing: 0.5px;
    }}
    .badge-crit {{ background: rgba(239, 68, 68, 0.2); color: #f87171; border: 1px solid #ef4444; }}
    .badge-warn {{ background: rgba(245, 158, 11, 0.2); color: #fbbf24; border: 1px solid #f59e0b; }}
    .badge-info {{ background: rgba(59, 130, 246, 0.2); color: #60a5fa; border: 1px solid #3b82f6; }}

    .recommendations-box {{
        display: flex;
        flex-direction: column;
        gap: 16px;
        margin-bottom: 40px;
    }}
    .recommendation-item {{
        background: var(--bg-surface);
        border: 1px solid var(--border-color);
        border-left: 4px solid var(--accent-rose);
        border-radius: 10px;
        padding: 20px;
    }}
    .rec-header {{ display: flex; align-items: center; gap: 12px; margin-bottom: 10px; }}
    .rec-header h3 {{ font-size: 17px; font-weight: 600; }}
    .subsystem-pill {{ background: #1f2937; padding: 4px 8px; border-radius: 4px; font-size: 12px; color: #9ca3af; }}
    .root-cause {{ color: #d1d5db; margin-bottom: 12px; font-size: 14px; }}
    .fix-list {{ padding-left: 20px; font-size: 14px; color: #e5e7eb; }}
    .fix-list li {{ margin-bottom: 6px; }}

    footer {{
        text-align: center;
        padding-top: 20px;
        border-top: 1px solid var(--border-color);
        color: var(--text-dim);
        font-size: 13px;
    }}
</style>
</head>
<body>
<div class="container">
    <header>
        <div class="brand">
            <h1>🛡️ Physics Engine Diagnostic Inspector</h1>
            <p>Automated Verification & Deep Telemetry Analysis | Target: <strong>{test_name}</strong> | Backend: <strong>{backend_name}</strong></p>
        </div>
        <div class="score-badge">
            <div class="score-val">{score}/100</div>
            <div class="score-grade">{grade}</div>
        </div>
    </header>

    <div class="metrics-grid">
        {meta_html}
        <div class="metric-card">
            <div class="metric-label">Anomalies Detected</div>
            <div class="metric-val">{len(anomalies)}</div>
        </div>
    </div>

    <h2>🎞️ Visual Evidence & Replays</h2>
    {gif_html}
    <div class="media-section">
        {screenshots_html}
    </div>

    <h2>⚠️ Detected Physical Defects & Instabilities</h2>
    <div class="table-container">
        <table>
            <thead>
                <tr>
                    <th>Severity</th>
                    <th>Anomaly Code</th>
                    <th>Timestamp</th>
                    <th>Metric / Threshold</th>
                    <th>Physical Root Cause & Impact</th>
                </tr>
            </thead>
            <tbody>
                {anom_rows}
            </tbody>
        </table>
    </div>

    <h2>🔧 Prescribed Fixes & Engineering Actions</h2>
    <div class="recommendations-box">
        {recs_html}
    </div>

    <footer>
        GameLib Physics Engine Automated Inspector &middot; Native DLL FFI Diagnostic Suite
    </footer>
</div>
</body>
</html>
"""
        return html
