import { Bar } from "react-chartjs-2";
import {
  Chart as ChartJS,
  CategoryScale,
  LinearScale,
  BarElement,
  Title,
  Tooltip,
  Legend,
} from "chart.js";

ChartJS.register(
  CategoryScale,
  LinearScale,
  BarElement,
  Title,
  Tooltip,
  Legend
);

const ESGBarChart = ({ data, ticker }) => {
  const chartData = {
    labels: ticker.map((t, index) => t || `Company ${index + 1}`),
    datasets: [
      {
        label: "Environmental",
        data: data.map((item) => item.e),
        backgroundColor: "#16a34a",
      },
      {
        label: "Social",
        data: data.map((item) => item.s),
        backgroundColor: "#eab308",
      },
      {
        label: "Governance",
        data: data.map((item) => item.g),
        backgroundColor: "#7c3aed",
      },
    ],
  };

  const options = {
    scales: {
      x: {
        stacked: true,
        ticks: {
          color: "#d1d5db", // gray-300 for x-axis text
        },
        grid: {
          color: "#6b7280", // gray-500 grid lines
        },
      },
      y: {
        stacked: true,
        min: 0,
        max: 100,
        ticks: {
          stepSize: 10,
          color: [
            "#22c55e",
            "#166534",
            "#fbbf24",
            "#fdba74",
            "red",
            "red",
            "red",
            "red",
            "red",
            "red",
            "red",
          ], // Color coding the Y axis
        },
        grid: {
          color: "#6b7280", // gray-500 grid lines
        },
        afterBuildTicks: (axis) => {
          axis.ticks.forEach((tick) => {
            if (tick.value >= 30) {
              tick.color = "red";
            } else if (tick.value >= 20) {
              tick.color = "orange";
            } else if (tick.value >= 10) {
              tick.color = "yellow";
            } else {
              tick.color = "green";
            }
          });
        },
      },
    },
    plugins: {
      legend: {
        labels: {
          color: "#d1d5db", // gray-300 for legend
        },
      },
      tooltip: {
        enabled: true,
      },
      beforeDraw: (chart) => {
        const {
          ctx,
          chartArea: { left, right },
          scales: { y },
        } = chart;
        ctx.save();

        const drawDashedLine = (value) => {
          ctx.setLineDash([5, 5]);
          ctx.beginPath();
          ctx.moveTo(left, y.getPixelForValue(value));
          ctx.lineTo(right, y.getPixelForValue(value));
          ctx.strokeStyle = "rgba(0, 0, 0, 0.5)";
          ctx.lineWidth = 1;
          ctx.stroke();
          ctx.closePath();
        };

        [10, 20, 30].forEach(drawDashedLine);
        ctx.restore();
      },
    },
    maintainAspectRatio: false,
  };

  return (
    <div
      style={{
        width: "100%",
        height: "400px",
        border: "1px solid #6b7280", // gray-500 border
        borderRadius: "0.5rem",
        padding: "1rem",
      }}
    >
      <Bar data={chartData} options={options} />
    </div>
  );
};

export default ESGBarChart;
