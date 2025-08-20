import { NextResponse } from "next/server";
import {
  getAzureOpenAIClient,
  AZURE_CONFIG,
} from "../../../../lib/azureOpenAI";
export const maxDuration = 60; // This function can run for a maximum of 5 seconds
export const dynamic = "force-dynamic";

export async function POST(request) {
  try {
    const { fullText, firstPageText } = await request.json();
    const client = getAzureOpenAIClient();

    const completion = await client.chat.completions.create({
      model: AZURE_CONFIG.deployment,
      messages: [
        {
          role: "system",
          content:
            "You have two tasks: 1. Determine the title of the research paper, 2. Determine the year the paper was published.",
        },
        {
          role: "user",
          content: `Analyze the following text: ${firstPageText}. Your response should be two parts separated by a comma. The first part is the title, the second part is the year. Do not include any other words in your response. The title should be extracted from the header or title section, and the year should be extracted from the publication information.`,
        },
      ],
    });

    const [tickerInfo, fiscalYearInfo] = completion.choices[0].message.content
      .split(",")
      .map((s) => s.trim());

    const summaryCompletion = await client.chat.completions.create({
      model: AZURE_CONFIG.deployment,
      messages: [
        {
          role: "system",
          content:
            "You are a research paper analyst and summarizer. Your task is to read the section provided to you from a research paper and create a comprehensive, detailed summary of the content. For each main topic presented in the text, break it down into subtopics and thoroughly explain each one. Your summary should capture the essence of the research, ensuring clarity and depth of understanding. Key Instructions:  1. Identify the main topics covered in the section and break them down into relevant subtopics. Provide clear and concise explanations for each subtopic, ensuring that the reader can understand the flow and structure of the research.2. For every concept, term, or technical jargon mentioned, offer a detailed explanation. Ensure that readers unfamiliar with the topic can understand the terminology. If a specific theory, model, or framework is referenced, provide background information and context to make it clear why it is important to the research.3. Whenever numerical results, statistics, percentages, or scores are presented, explain their significance in detail. Discuss whether these numbers are surprising, expected, or have any implications for the field of study. Provide context on how these results compare to other similar studies, if relevant.4. Summarize the methodologies used in the research, explaining the steps, procedures, or techniques involved. If the section discusses experimental setups, sample sizes, or data collection methods, describe these thoroughly and explain their relevance to the study's outcomes. Discuss the strengths and potential weaknesses of the methodology, if evident in the text.5. Highlight the key findings of the section and explain their importance within the broader context of the field. Discuss the implications of these findings—how do they contribute to the existing body of knowledge? Are there any practical applications, policy recommendations, or future research directions suggested? If the section contains conclusions or interpretations, summarize them and provide insight into their significance.6. Use headers and subheaders to organize your summary clearly. Use bullet points to list key information, findings, or points. Use bold text for emphasis on critical terms, findings, or concepts that are central to the research. Ensure all lists and bullet points are properly formatted and maintain consistency throughout the summary. 7. Stick closely to the content presented in the section without adding new topics or creating sections that do not exist in the original text. This approach will help ensure your summary is thorough, informative, and accessible to both experts and those new to the topic. If the response calls for a mathematical equation, then wrap the equation in two dollar signs ($$) with no space between the first character and the dollar signs and the last character and the dollar signs to ensure proper formatting.",
        },
        {
          role: "user",
          content: `The information in brackets is the research data you are summarizing. You will read the section given to you and summarize it using the Tree of Thought method Generate a title. Break down the main topics into subtopics and summarize each part with detailed explanations. Include important findings, methodologies, and implications. Use clear and comprehensive language. Use markdown format. Analyze the numerical results presented, explaining their significance and discussing whether they are surprising or meaningful.[${fullText}]`,
        },
      ],
    });

    const summary = summaryCompletion.choices[0].message.content;
    console.log(summary);
    return NextResponse.json({ summary, tickerInfo }, { status: 200 });
  } catch (error) {
    console.error("Error occurred while processing the request:", error);

    let errorMessage = "Internal Server Error";
    if (error instanceof SyntaxError) {
      errorMessage = "Invalid JSON format";
    } else if (error.response) {
      errorMessage = `API Error: ${error.response.statusText}`;
    }

    return NextResponse.json({ error: errorMessage }, { status: 500 });
  }
}
