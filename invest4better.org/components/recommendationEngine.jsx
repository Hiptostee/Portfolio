import ReactMarkdown from "react-markdown";
import remarkGfm from "remark-gfm";
import styles from "../app/TypingText.module.css";

export default function RecommendationEngine({
  onGenerate,
  loadingGPT,
  recommendation,
  recommendationRef,
  disabled,
}) {
  return (
    <div className="mb-4">
      {!recommendation && (
        <div className="flex items-center justify-center">
          <button
            onClick={onGenerate}
            className="rounded-md bg-green-700 px-6 py-3 text-white hover:bg-green-800 disabled:cursor-not-allowed disabled:bg-gray-600"
            disabled={disabled}
          >
            {loadingGPT ? "Generating..." : "Get Recommendation"}
          </button>
        </div>
      )}
      {recommendation && (
        <div
          ref={recommendationRef}
          className={`rounded-lg bg-gray-800 p-4 text-gray-200 ${styles.typingContainer}`}
        >
          <h3 className="mb-2 text-lg font-semibold text-green-500">
            Analyst Recommendation
          </h3>
          <div className="prose prose-invert max-w-none">
            <ReactMarkdown remarkPlugins={[remarkGfm]}>
              {recommendation}
            </ReactMarkdown>
          </div>
        </div>
      )}
    </div>
  );
}
